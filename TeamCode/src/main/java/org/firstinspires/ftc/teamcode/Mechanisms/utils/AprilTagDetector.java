package org.firstinspires.ftc.teamcode.Mechanisms.utils;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDetector {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // Camera configuration
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    // Vision components
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Detection state
    private List<AprilTagDetection> currentDetections;
    private boolean initialized = false;

    public AprilTagDetector(HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);
    }

    /**
     * Initialize the AprilTag processor and Vision Portal
     */
    private void initAprilTag(HardwareMap hardwareMap) {
        try {
            // Create the AprilTag processor
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawTagOutline(true)
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build();

            // Adjust detection parameters for better performance
            aprilTag.setDecimation(2); // Balance between range and frame rate

            // Create the vision portal
            VisionPortal.Builder builder = new VisionPortal.Builder();

            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            // Set camera resolution for better detection
            // builder.setCameraResolution(new Size(640, 480));

            builder.addProcessor(aprilTag);
            builder.enableLiveView(false); // Disable live view for better performance

            visionPortal = builder.build();

            // Wait for camera to initialize
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            initialized = true;

        } catch (Exception e) {
            System.err.println("AprilTagDetector initialization failed: " + e.getMessage());
            initialized = false;
        }
    }

    /**
     * Update detections - call this in your loop
     */
    public void update() {
        if (initialized && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            currentDetections = aprilTag.getDetections();
        }
    }

    /**
     * Get distance to a specific AprilTag
     * @param tagId The ID of the AprilTag to detect
     * @return Distance in inches, or -1 if not found
     */
    public double getDistanceToTag(int tagId) {
        if (!initialized || currentDetections == null) {
            return -1;
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagId) {
                return detection.ftcPose.range;
            }
        }
        return -1;
    }

    /**
     * Get distance to the closest backboard AprilTag (IDs 1, 2, 3 for blue, 4, 5, 6 for red)
     * @param isBlueAlliance Whether we're on blue alliance (uses tags 1,2,3) or red (uses tags 4,5,6)
     * @return Distance in inches, or -1 if no valid tag found
     */
    public double getBackboardDistance(boolean isBlueAlliance) {
        if (!initialized || currentDetections == null) {
            return -1;
        }

        int[] targetTags = isBlueAlliance ? new int[]{1, 2, 3} : new int[]{4, 5, 6};
        double closestDistance = -1;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                for (int tagId : targetTags) {
                    if (detection.id == tagId) {
                        double distance = detection.ftcPose.range;
                        if (closestDistance == -1 || distance < closestDistance) {
                            closestDistance = distance;
                        }
                    }
                }
            }
        }
        return closestDistance;
    }

    /**
     * Get the robot's pose relative to a specific AprilTag
     * @param tagId The ID of the AprilTag
     * @return The robot pose, or null if not found
     */
    public AprilTagDetection getTagDetection(int tagId) {
        if (!initialized || currentDetections == null) {
            return null;
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Check if any AprilTags are currently detected
     */
    public boolean hasDetections() {
        return initialized && currentDetections != null && !currentDetections.isEmpty();
    }

    /**
     * Get the number of AprilTags currently detected
     */
    public int getDetectionCount() {
        return (initialized && currentDetections != null) ? currentDetections.size() : 0;
    }

    /**
     * Get all current detections
     */
    public List<AprilTagDetection> getDetections() {
        return currentDetections;
    }

    /**
     * Add telemetry data for debugging
     */
    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry telemetry) {
        if (!initialized) {
            telemetry.addLine("AprilTagDetector: NOT INITIALIZED");
            return;
        }

        telemetry.addData("# AprilTags Detected", getDetectionCount());
        telemetry.addData("Camera State", visionPortal.getCameraState().toString());

        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z));
                    telemetry.addLine(String.format("Distance: %6.1f inches", detection.ftcPose.range));
                    telemetry.addLine(String.format("Bearing: %6.1f degrees", detection.ftcPose.bearing));
                    telemetry.addLine(String.format("Yaw: %6.1f degrees", detection.ftcPose.yaw));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                }
            }
        }

        // Add quick reference for common tags
        telemetry.addLine("\n=== Backboard Tags ===");
        telemetry.addLine("Blue: 1,2,3 | Red: 4,5,6");
        telemetry.addLine("=== Audience Tags ===");
        telemetry.addLine("Blue: 9,10 | Red: 19,20");
    }

    /**
     * Stop the vision portal and release resources
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
        initialized = false;
    }

    /**
     * Check if the detector is properly initialized
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Set camera pose parameters (for advanced calibration)
     */
    public void setCameraPose(Position position, YawPitchRollAngles orientation) {
        this.cameraPosition = position;
        this.cameraOrientation = orientation;
        // Note: This won't update the running processor, would need to recreate
    }
}