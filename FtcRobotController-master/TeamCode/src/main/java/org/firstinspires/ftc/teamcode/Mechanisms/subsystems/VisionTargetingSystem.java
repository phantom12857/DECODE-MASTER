// NEW: VisionTargetingSystem.java
package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Mechanisms.core.PIDController;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.GamePiece;

import java.util.List;

/**
 * VisionTargetingSystem handles AprilTag detection and turret auto-aiming.
 *
 * Features:
 * - Detects AprilTags on goal baskets
 * - Compensates for camera offset from turret center (right-mounted camera)
 * - Adjusts for varying distances using tag size
 * - Controls turret to center on target
 * - Interprets "monolith" tags to determine ball firing order
 *
 * Camera: 720p Logitech, mounted on right side of turret
 * Turret: ±70° rotation range (NO full rotation)
 */
public class VisionTargetingSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================

    // Camera Configuration
    private static final int CAMERA_WIDTH = 1280;  // 720p width
    private static final int CAMERA_HEIGHT = 720;   // 720p height
    private static final double CAMERA_OFFSET_INCHES = 3.5;  // Camera is 3.5" right of turret center
    private static final double CAMERA_FOV_HORIZONTAL = 70.0;  // Logitech C270 approximate FOV

    // Turret Limits
    private static final double TURRET_MAX_LEFT = 70.0;   // Degrees
    private static final double TURRET_MAX_RIGHT = 70.0;  // Degrees

    // AprilTag IDs (customize based on your game)
    private static final int BLUE_BASKET_TAG_ID = 11;
    private static final int RED_BASKET_TAG_ID = 12;
    private static final int MONOLITH_PURPLE_FIRST = 13;  // Fire purple balls first
    private static final int MONOLITH_GREEN_FIRST = 14;   // Fire green balls first
    private static final int MONOLITH_ANY_ORDER = 15;     // Fire in any order

    // Tag Physical Properties
    private static final double TAG_SIZE_INCHES = 4.0;  // Standard FTC AprilTag size

    // PID Tuning
    private static final double KP_TURRET = 0.02;  // Proportional gain
    private static final double KI_TURRET = 0.0;   // Integral gain
    private static final double KD_TURRET = 0.003; // Derivative gain
    private static final double CENTERING_TOLERANCE_PIXELS = 20.0;

    // Timeouts
    private static final double LOCK_ON_TIMEOUT_S = 3.0;

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final TurretSystem turret;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // ==================================================
    // S T A T E
    // ==================================================
    private final PIDController turretPID;
    private TargetingState currentState = TargetingState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private AprilTagDetection targetTag = null;
    private int targetTagId = -1;
    private boolean isBlueAlliance = true;
    
    // Performance optimization: enable/disable vision processing
    private boolean enabled = false;

    private GamePiece.Color[] desiredFiringOrder = null;  // Determined from monolith
    private String lastError = "";
    
    // Pre-calculated constants for performance
    private static final double PIXELS_PER_DEGREE = CAMERA_WIDTH / CAMERA_FOV_HORIZONTAL;

    /**
     * Targeting state machine
     */
    public enum TargetingState {
        IDLE,               // Not targeting
        SEARCHING,          // Looking for AprilTag
        CALCULATING,        // Computing turret adjustment
        ROTATING,           // Rotating turret to target
        LOCKED,             // Locked onto target
        LOST_TRACK,         // Lost sight of target
        ERROR               // Something went wrong
    }

    /**
     * Firing order priority based on monolith tag
     */
    public enum FiringPriority {
        PURPLE_FIRST,       // Shoot purple balls before green
        GREEN_FIRST,        // Shoot green balls before purple
        ANY_ORDER,          // Order doesn't matter
        UNKNOWN             // Haven't seen monolith yet
    }

    private FiringPriority currentPriority = FiringPriority.UNKNOWN;

    /**
     * Constructor for VisionTargetingSystem.
     *
     * @param hardwareMap The robot's hardware map
     * @param turret Reference to the turret subsystem for control
     */
    public VisionTargetingSystem(HardwareMap hardwareMap, TurretSystem turret) {
        this.turret = turret;
        this.turretPID = new PIDController(KP_TURRET, KI_TURRET, KD_TURRET);

        try {
            // Initialize AprilTag processor
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .build();

            // Initialize vision portal
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new android.util.Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                    .addProcessor(aprilTagProcessor)
                    .build();

        } catch (Exception e) {
            throw new RuntimeException("Vision targeting system initialization failed", e);
        }
    }

    @Override
    public void update() {
        // Skip processing if disabled or in inactive states
        if (!enabled || currentState == TargetingState.IDLE || currentState == TargetingState.ERROR) {
            return;
        }

        try {
            switch (currentState) {
                case SEARCHING:
                    updateSearching();
                    break;
                case CALCULATING:
                    updateCalculating();
                    break;
                case ROTATING:
                    updateRotating();
                    break;
                case LOCKED:
                    updateLocked();
                    break;
                case LOST_TRACK:
                    updateLostTrack();
                    break;
            }
        } catch (Exception e) {
            handleError("Update error: " + e.getMessage());
            transitionTo(TargetingState.ERROR);
        }
    }
    
    /**
     * Enable or disable vision processing.
     * When disabled, update() returns immediately to save CPU.
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled && currentState != TargetingState.IDLE) {
            cancelTargeting();
        }
    }
    
    /**
     * Check if vision targeting is currently enabled.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Initiates targeting on the appropriate basket tag based on alliance.
     *
     * @return true if targeting started, false if already busy
     */
    public boolean startTargeting() {
        if (currentState != TargetingState.IDLE) {
            return false;
        }

        targetTagId = isBlueAlliance ? BLUE_BASKET_TAG_ID : RED_BASKET_TAG_ID;
        transitionTo(TargetingState.SEARCHING);
        return true;
    }

    /**
     * Scans for the monolith tag to determine firing order.
     * Call this during autonomous or at start of TeleOp.
     *
     * @return true if monolith found and priority determined
     */
    public boolean scanForMonolith() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == MONOLITH_PURPLE_FIRST) {
                currentPriority = FiringPriority.PURPLE_FIRST;
                desiredFiringOrder = new GamePiece.Color[]{
                        GamePiece.Color.PURPLE,
                        GamePiece.Color.PURPLE,
                        GamePiece.Color.GREEN
                };
                return true;
            } else if (detection.id == MONOLITH_GREEN_FIRST) {
                currentPriority = FiringPriority.GREEN_FIRST;
                desiredFiringOrder = new GamePiece.Color[]{
                        GamePiece.Color.GREEN,
                        GamePiece.Color.GREEN,
                        GamePiece.Color.PURPLE
                };
                return true;
            } else if (detection.id == MONOLITH_ANY_ORDER) {
                currentPriority = FiringPriority.ANY_ORDER;
                desiredFiringOrder = null;  // Fire in any order
                return true;
            }
        }

        return false;  // Monolith not found
    }

    /**
     * Cancels targeting and returns to idle.
     */
    public void cancelTargeting() {
        transitionTo(TargetingState.IDLE);
        targetTag = null;
        turret.setPower(0);
    }

    // ==================================================
    // S T A T E   U P D A T E   M E T H O D S
    // ==================================================

    private void updateSearching() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        // Look for our target tag
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId) {
                targetTag = detection;
                transitionTo(TargetingState.CALCULATING);
                return;
            }
        }

        // Timeout check
        if (stateTimer.seconds() > LOCK_ON_TIMEOUT_S) {
            handleError("Target tag " + targetTagId + " not found");
            transitionTo(TargetingState.ERROR);
        }
    }

    private void updateCalculating() {
        if (targetTag == null) {
            transitionTo(TargetingState.SEARCHING);
            return;
        }

        // Get tag center in image coordinates
        double tagCenterX = targetTag.center.x;
        double imageCenterX = CAMERA_WIDTH / 2.0;

        // Calculate pixel error (positive = tag is to the right)
        double pixelError = tagCenterX - imageCenterX;

        // Convert pixel error to angular error
        double angularErrorDeg = pixelErrorToAngle(pixelError);

        // Compensate for camera offset (camera is right of turret center)
        double compensatedError = compensateForCameraOffset(angularErrorDeg, targetTag);

        // Check if we're already centered
        if (Math.abs(pixelError) < CENTERING_TOLERANCE_PIXELS) {
            transitionTo(TargetingState.LOCKED);
        } else {
            // Apply PID to calculate turret adjustment
            double turretAdjustment = turretPID.calculate(0, compensatedError);

            // Check turret limits
            double currentAngle = turret.getCurrentAngleDegrees();
            double targetAngle = currentAngle + turretAdjustment;

            if (targetAngle < -TURRET_MAX_LEFT) {
                targetAngle = -TURRET_MAX_LEFT;
                handleError("Turret limit reached (left)");
            } else if (targetAngle > TURRET_MAX_RIGHT) {
                targetAngle = TURRET_MAX_RIGHT;
                handleError("Turret limit reached (right)");
            }

            // Command turret to rotate
            turret.setTargetAngle(targetAngle);
            transitionTo(TargetingState.ROTATING);
        }
    }

    private void updateRotating() {
        // Check if turret reached target
        if (!turret.isBusy()) {
            // Re-check tag position
            transitionTo(TargetingState.SEARCHING);
        }

        // Timeout check
        if (stateTimer.seconds() > LOCK_ON_TIMEOUT_S) {
            handleError("Turret rotation timeout");
            transitionTo(TargetingState.ERROR);
        }
    }

    private void updateLocked() {
        // Continuously update to maintain lock
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        boolean foundTarget = false;
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId) {
                targetTag = detection;
                foundTarget = true;

                // Check if still centered
                double pixelError = Math.abs(targetTag.center.x - (CAMERA_WIDTH / 2.0));
                if (pixelError > CENTERING_TOLERANCE_PIXELS * 2) {
                    // Lost centering, need to re-acquire
                    transitionTo(TargetingState.CALCULATING);
                }
                break;
            }
        }

        if (!foundTarget) {
            transitionTo(TargetingState.LOST_TRACK);
        }
    }

    private void updateLostTrack() {
        // Try to re-acquire for a short time
        transitionTo(TargetingState.SEARCHING);
    }

    // ==================================================
    // C A L C U L A T I O N   M E T H O D S
    // ==================================================

    /**
     * Converts pixel error to angular error in degrees.
     *
     * @param pixelError Error in pixels (positive = right)
     * @return Angular error in degrees
     */
    private double pixelErrorToAngle(double pixelError) {
        // Use pre-calculated constant for better performance
        return pixelError / PIXELS_PER_DEGREE;
    }

    /**
     * Compensates for camera being offset from turret center.
     * Uses parallax correction based on estimated distance to target.
     *
     * @param angularError Initial angular error in degrees
     * @param detection AprilTag detection with range information
     * @return Compensated angular error in degrees
     */
    private double compensateForCameraOffset(double angularError, AprilTagDetection detection) {
        // Estimate distance from tag size in image
        double distanceInches = estimateDistance(detection);

        // Calculate parallax angle due to camera offset
        // tan(parallax) = offset / distance
        double parallaxAngle = Math.toDegrees(Math.atan2(CAMERA_OFFSET_INCHES, distanceInches));

        // Subtract parallax (camera is to the right, so actual target is more left)
        return angularError - parallaxAngle;
    }

    /**
     * Estimates distance to AprilTag based on its apparent size.
     *
     * @param detection AprilTag detection
     * @return Estimated distance in inches
     */
    private double estimateDistance(AprilTagDetection detection) {
        // If we have range from ftcPose, use it
        if (detection.ftcPose != null) {
            return detection.ftcPose.range;  // Already in inches
        }

        // Fallback: estimate from tag corners
        // Calculate tag width in pixels
        double tagWidthPixels = Math.abs(
                detection.corners[1].x - detection.corners[0].x
        );

        // Use pinhole camera model
        // distance = (realWidth * focalLength) / pixelWidth
        // focalLength ≈ (imageWidth / 2) / tan(FOV / 2)
        double focalLength = (CAMERA_WIDTH / 2.0) / Math.tan(Math.toRadians(CAMERA_FOV_HORIZONTAL / 2.0));
        double distanceInches = (TAG_SIZE_INCHES * focalLength) / tagWidthPixels;

        return distanceInches;
    }

    /**
     * Determines the optimal firing order based on current ball colors
     * and the desired firing priority from the monolith.
     *
     * @param ballColors Array of 3 colors representing current spindexer state
     * @return Indices to fire in optimal order, or null if order doesn't matter
     */
    public int[] getOptimalFiringOrder(GamePiece.Color[] ballColors) {
        if (currentPriority == FiringPriority.ANY_ORDER ||
                currentPriority == FiringPriority.UNKNOWN) {
            return null;  // Fire in current order (0, 1, 2)
        }

        // Count colors
        int purpleCount = 0;
        int greenCount = 0;
        for (GamePiece.Color color : ballColors) {
            if (color == GamePiece.Color.PURPLE) purpleCount++;
            else if (color == GamePiece.Color.GREEN) greenCount++;
        }

        // Build firing order based on priority
        int[] order = new int[3];
        int orderIndex = 0;

        // Determine which color to fire first
        GamePiece.Color firstColor = (currentPriority == FiringPriority.PURPLE_FIRST) ?
                GamePiece.Color.PURPLE : GamePiece.Color.GREEN;
        GamePiece.Color secondColor = (firstColor == GamePiece.Color.PURPLE) ?
                GamePiece.Color.GREEN : GamePiece.Color.PURPLE;

        // Add slots in priority order
        for (int i = 0; i < ballColors.length; i++) {
            if (ballColors[i] == firstColor) {
                order[orderIndex++] = i;
            }
        }
        for (int i = 0; i < ballColors.length; i++) {
            if (ballColors[i] == secondColor) {
                order[orderIndex++] = i;
            }
        }
        // Add any empty slots last
        for (int i = 0; i < ballColors.length; i++) {
            if (ballColors[i] == GamePiece.Color.NONE) {
                order[orderIndex++] = i;
            }
        }

        return order;
    }

    // ==================================================
    // S T A T E   M A N A G E M E N T
    // ==================================================

    private void transitionTo(TargetingState newState) {
        if (currentState == newState) return;

        currentState = newState;
        stateTimer.reset();

        switch (newState) {
            case IDLE:
                turret.setPower(0);
                turretPID.reset();
                break;
            case SEARCHING:
                // Slowly rotate turret to scan for tags
                break;
            case ERROR:
                turret.setPower(0);
                // Error logged in lastError, accessible via telemetry
                break;
        }
    }

    private void handleError(String errorMessage) {
        lastError = errorMessage;
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
        transitionTo(TargetingState.IDLE);
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addLine("--- Vision Targeting ---");
        telemetry.addData("State", currentState);
        telemetry.addData("Firing Priority", currentPriority);

        if (targetTag != null) {
            telemetry.addData("Target Tag", targetTagId);
            telemetry.addData("Tag Center X", "%.0f px", targetTag.center.x);
            telemetry.addData("Distance", "%.1f in", estimateDistance(targetTag));

            double pixelError = targetTag.center.x - (CAMERA_WIDTH / 2.0);
            telemetry.addData("Pixel Error", "%.0f px", pixelError);
        }

        if (currentState == TargetingState.ERROR) {
            telemetry.addData("⚠️ ERROR", lastError);
        }

        // Show detected tags
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        telemetry.addData("Tags Visible", detections.size());
    }

    // ==================================================
    // G E T T E R S   A N D   S E T T E R S
    // ==================================================

    public boolean isLocked() {
        return currentState == TargetingState.LOCKED;
    }

    public boolean isTargeting() {
        return currentState != TargetingState.IDLE && currentState != TargetingState.ERROR;
    }

    public TargetingState getState() {
        return currentState;
    }

    public FiringPriority getFiringPriority() {
        return currentPriority;
    }

    public void setAlliance(boolean isBlue) {
        this.isBlueAlliance = isBlue;
    }

    public String getLastError() {
        return lastError;
    }

    /**
     * Gets the estimated distance to the locked target.
     *
     * @return Distance in inches, or -1 if not locked
     */
    public double getTargetDistance() {
        if (targetTag != null) {
            return estimateDistance(targetTag);
        }
        return -1;
    }
}