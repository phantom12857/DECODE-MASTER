// NEW: MechanismCoordinator.java
package org.firstinspires.ftc.teamcode.Mechanisms.core;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.subsystems.LauncherSystem;
import org.firstinspires.ftc.teamcode.Mechanisms.subsystems.SpindexerSystem;
import org.firstinspires.ftc.teamcode.Mechanisms.subsystems.VisionTargetingSystem;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.GamePiece;

/**
 * Enhanced MechanismCoordinator with vision-guided targeting.
 *
 * Features:
 * - Synchronized spindexer/launcher firing
 * - Vision-guided turret auto-aim
 * - Intelligent ball sequencing based on monolith tags
 * - Multi-shot sequences with optimal firing order
 *
 * USAGE:
 * <code>
 * // Initialize
 * MechanismCoordinator coordinator = new MechanismCoordinator(
 *     spindexer, launcher, visionTargeting);
 *
 * // Scan for monolith at start
 * coordinator.scanForFiringPriority();
 *
 * // In your OpMode loop:
 * coordinator.update();
 *
 * // Vision-guided auto-fire:
 * coordinator.startVisionGuidedFire(3);  // Fire 3 balls with auto-aim
 * </code>
 */
public class MechanismCoordinator {

    // ==================================================
    // D E P E N D E N C I E S
    // ==================================================
    private final SpindexerSystem spindexer;
    private final LauncherSystem launcher;
    private final VisionTargetingSystem visionTargeting;  // Optional

    // ==================================================
    // F I R I N G   S E Q U E N C E   S T A T E
    // ==================================================
    public enum FireSequenceState {
        IDLE,                       // Not firing
        VISION_TARGETING,           // Locking onto AprilTag
        WAIT_FOR_VISION_LOCK,       // Waiting for turret to center
        WAIT_FOR_LAUNCHER_READY,    // Waiting for launcher to reach target RPM
        ROTATING_SPINDEXER,         // Spindexer moving to position
        WAIT_FOR_SPINDEXER,         // Waiting for spindexer to reach position
        FIRING_KICKER,              // Kicker extending to push ball
        WAIT_FOR_KICKER_COMPLETE,   // Waiting for kicker to retract
        COOLDOWN,                   // Brief pause before next shot
        COMPLETE,                   // Sequence finished successfully
        ERROR                       // Something went wrong
    }

    private FireSequenceState currentState = FireSequenceState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private String lastError = "";

    // ==================================================
    // F I R I N G   M O D E
    // ==================================================
    public enum FiringMode {
        MANUAL,         // Manual control (no coordinator)
        SYNCHRONIZED,   // Basic synchronized firing
        VISION_GUIDED   // Vision-guided with auto-aim
    }

    private FiringMode currentMode = FiringMode.SYNCHRONIZED;

    // ==================================================
    // M U L T I - S H O T   S T A T E
    // ==================================================
    private int[] firingSequence = null;  // Order of slots to fire (e.g., [2, 0, 1])
    private int currentShotIndex = 0;
    private int totalShots = 0;
    private int shotsFired = 0;
    private boolean useVisionGuidedAiming = false;

    // ==================================================
    // C O N F I G U R A B L E   T I M E O U T S
    // ==================================================
    private static final double VISION_LOCK_TIMEOUT_S = 4.0;
    private static final double LAUNCHER_READY_TIMEOUT_S = 3.0;
    private static final double SPINDEXER_MOVE_TIMEOUT_S = 2.0;
    private static final double KICKER_COMPLETE_TIMEOUT_S = 1.5;
    private static final double COOLDOWN_TIME_S = 0.2;

    /**
     * Constructor for MechanismCoordinator without vision.
     *
     * @param spindexer The spindexer subsystem
     * @param launcher  The launcher subsystem
     */
    public MechanismCoordinator(SpindexerSystem spindexer, LauncherSystem launcher) {
        this(spindexer, launcher, null);
    }

    /**
     * Constructor for MechanismCoordinator with vision targeting.
     *
     * @param spindexer       The spindexer subsystem
     * @param launcher        The launcher subsystem
     * @param visionTargeting The vision targeting subsystem (optional, can be null)
     */
    public MechanismCoordinator(SpindexerSystem spindexer, LauncherSystem launcher,
                                VisionTargetingSystem visionTargeting) {
        this.spindexer = spindexer;
        this.launcher = launcher;
        this.visionTargeting = visionTargeting;
    }

    /**
     * Update method - must be called in OpMode loop.
     */
    public void update() {
        if (currentState == FireSequenceState.IDLE ||
                currentState == FireSequenceState.COMPLETE) {
            return;
        }

        switch (currentState) {
            case VISION_TARGETING:
                updateVisionTargeting();
                break;
            case WAIT_FOR_VISION_LOCK:
                updateWaitForVisionLock();
                break;
            case WAIT_FOR_LAUNCHER_READY:
                updateWaitForLauncherReady();
                break;
            case ROTATING_SPINDEXER:
                updateRotatingSpindexer();
                break;
            case WAIT_FOR_SPINDEXER:
                updateWaitForSpindexer();
                break;
            case FIRING_KICKER:
                updateFiringKicker();
                break;
            case WAIT_FOR_KICKER_COMPLETE:
                updateWaitForKickerComplete();
                break;
            case COOLDOWN:
                updateCooldown();
                break;
            case ERROR:
                // Stay in error state until reset
                break;
        }
    }

    /**
     * Scans for monolith tag and sets firing priority.
     * Call this during autonomous init or at start of TeleOp.
     *
     * @return true if monolith found
     */
    public boolean scanForFiringPriority() {
        if (visionTargeting != null) {
            return visionTargeting.scanForMonolith();
        }
        return false;
    }

    /**
     * Initiates a single synchronized shot.
     *
     * @return true if sequence started
     */
    public boolean fireSingleShot() {
        return startFiringSequence(1, false);
    }

    /**
     * Initiates a multi-shot sequence with intelligent ball ordering.
     *
     * @param shots Number of shots to fire (1-3)
     * @return true if sequence started
     */
    public boolean startMultiShotSequence(int shots) {
        return startFiringSequence(shots, false);
    }

    /**
     * Initiates vision-guided firing with auto-aim.
     *
     * @param shots Number of shots to fire (1-3)
     * @return true if sequence started
     */
    public boolean startVisionGuidedFire(int shots) {
        if (visionTargeting == null) {
            handleError("Vision targeting not available");
            return false;
        }
        return startFiringSequence(shots, true);
    }

    /**
     * Internal method to start a firing sequence.
     *
     * @param shots Number of shots
     * @param useVision Whether to use vision-guided aiming
     * @return true if started
     */
    private boolean startFiringSequence(int shots, boolean useVision) {
        if (!isIdle()) {
            return false;
        }

        if (shots < 1 || shots > 3) {
            handleError("Invalid shot count: " + shots);
            return false;
        }

        // Get current ball colors from spindexer
        GamePiece.Color[] ballColors = spindexer.getSlotColors();

        // Determine optimal firing order
        if (visionTargeting != null &&
                visionTargeting.getFiringPriority() != VisionTargetingSystem.FiringPriority.UNKNOWN) {
            // Use intelligent sequencing based on monolith
            firingSequence = visionTargeting.getOptimalFiringOrder(ballColors);
        } else {
            // Fire in current order
            firingSequence = new int[]{0, 1, 2};
        }

        // Limit to requested shot count
        totalShots = Math.min(shots, countAvailableBalls(ballColors));
        currentShotIndex = 0;
        shotsFired = 0;
        useVisionGuidedAiming = useVision;

        // Start sequence
        if (useVisionGuidedAiming) {
            currentMode = FiringMode.VISION_GUIDED;
            transitionTo(FireSequenceState.VISION_TARGETING);
        } else {
            currentMode = FiringMode.SYNCHRONIZED;
            transitionTo(FireSequenceState.WAIT_FOR_LAUNCHER_READY);
        }

        return true;
    }

    /**
     * Counts how many balls are actually loaded.
     */
    private int countAvailableBalls(GamePiece.Color[] ballColors) {
        int count = 0;
        for (GamePiece.Color color : ballColors) {
            if (color != GamePiece.Color.NONE) {
                count++;
            }
        }
        return count;
    }

    /**
     * Cancels the current firing sequence.
     */
    public void cancel() {
        if (visionTargeting != null) {
            visionTargeting.cancelTargeting();
        }
        transitionTo(FireSequenceState.IDLE);
        currentShotIndex = 0;
    }

    /**
     * Resets from error state.
     */
    public void reset() {
        lastError = "";
        currentShotIndex = 0;
        shotsFired = 0;
        transitionTo(FireSequenceState.IDLE);
    }

    // ==================================================
    // S T A T E   U P D A T E   M E T H O D S
    // ==================================================

    private void updateVisionTargeting() {
        if (!visionTargeting.isTargeting()) {
            // Start targeting
            visionTargeting.startTargeting();
        }
        transitionTo(FireSequenceState.WAIT_FOR_VISION_LOCK);
    }

    private void updateWaitForVisionLock() {
        if (visionTargeting.isLocked()) {
            // Locked on target, proceed to firing
            transitionTo(FireSequenceState.WAIT_FOR_LAUNCHER_READY);
        } else if (visionTargeting.getState() == VisionTargetingSystem.TargetingState.ERROR) {
            handleError("Vision targeting failed: " + visionTargeting.getLastError());
            transitionTo(FireSequenceState.ERROR);
        } else if (stateTimer.seconds() > VISION_LOCK_TIMEOUT_S) {
            handleError("Vision lock timeout");
            transitionTo(FireSequenceState.ERROR);
        }
    }

    private void updateWaitForLauncherReady() {
        if (launcher.isAtSpeed(100)) {
            // Launcher ready, move spindexer to correct position
            int targetSlot = firingSequence[currentShotIndex];
            if (spindexer.getCurrentStep() != targetSlot) {
                spindexer.moveToStep(targetSlot);
                transitionTo(FireSequenceState.ROTATING_SPINDEXER);
            } else {
                // Already in position, fire immediately
                transitionTo(FireSequenceState.FIRING_KICKER);
            }
        } else if (stateTimer.seconds() > LAUNCHER_READY_TIMEOUT_S) {
            handleError("Launcher failed to reach target speed");
            transitionTo(FireSequenceState.ERROR);
        }
    }

    private void updateRotatingSpindexer() {
        transitionTo(FireSequenceState.WAIT_FOR_SPINDEXER);
    }

    private void updateWaitForSpindexer() {
        if (!spindexer.isBusy()) {
            // Spindexer in position, fire!
            transitionTo(FireSequenceState.FIRING_KICKER);
        } else if (stateTimer.seconds() > SPINDEXER_MOVE_TIMEOUT_S) {
            handleError("Spindexer movement timeout");
            transitionTo(FireSequenceState.ERROR);
        }
    }

    private void updateFiringKicker() {
        transitionTo(FireSequenceState.WAIT_FOR_KICKER_COMPLETE);
    }

    private void updateWaitForKickerComplete() {
        if (!launcher.isKicking()) {
            // Kicker completed
            shotsFired++;
            currentShotIndex++;

            // Clear the slot we just fired from
            int firedSlot = firingSequence[currentShotIndex - 1];
            spindexer.clearSlot(firedSlot);

            if (currentShotIndex >= totalShots) {
                // All shots fired
                transitionTo(FireSequenceState.COMPLETE);
            } else {
                // More shots to fire
                transitionTo(FireSequenceState.COOLDOWN);
            }
        } else if (stateTimer.seconds() > KICKER_COMPLETE_TIMEOUT_S) {
            handleError("Kicker did not complete");
            // Continue anyway
            currentShotIndex++;
            if (currentShotIndex >= totalShots) {
                transitionTo(FireSequenceState.COMPLETE);
            } else {
                transitionTo(FireSequenceState.COOLDOWN);
            }
        }
    }

    private void updateCooldown() {
        if (stateTimer.seconds() > COOLDOWN_TIME_S) {
            // Cooldown complete, prepare next shot
            transitionTo(FireSequenceState.WAIT_FOR_LAUNCHER_READY);
        }
    }

    // ==================================================
    // S T A T E   M A N A G E M E N T
    // ==================================================

    private void transitionTo(FireSequenceState newState) {
        if (currentState == newState) return;

        currentState = newState;
        stateTimer.reset();

        switch (newState) {
            case FIRING_KICKER:
                launcher.kick();
                break;
            case IDLE:
            case COMPLETE:
                if (visionTargeting != null && useVisionGuidedAiming) {
                    visionTargeting.cancelTargeting();
                }
                break;
            case ERROR:
                System.err.println("MechanismCoordinator ERROR: " + lastError);
                if (visionTargeting != null) {
                    visionTargeting.cancelTargeting();
                }
                break;
        }
    }

    private void handleError(String errorMessage) {
        lastError = errorMessage;
    }

    // ==================================================
    // S T A T U S   M E T H O D S
    // ==================================================

    public boolean isIdle() {
        return currentState == FireSequenceState.IDLE ||
                currentState == FireSequenceState.COMPLETE;
    }

    public boolean isBusy() {
        return !isIdle() && currentState != FireSequenceState.ERROR;
    }

    public boolean isInError() {
        return currentState == FireSequenceState.ERROR;
    }

    public FireSequenceState getState() {
        return currentState;
    }

    public FiringMode getMode() {
        return currentMode;
    }

    public String getLastError() {
        return lastError;
    }

    public int getShotsFired() {
        return shotsFired;
    }

    public int getTotalShots() {
        return totalShots;
    }

    /**
     * Adds telemetry data for debugging.
     */
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addLine("--- Mechanism Coordinator ---");
        telemetry.addData("Mode", currentMode);
        telemetry.addData("State", currentState);

        if (isBusy()) {
            telemetry.addData("Shots", "%d/%d", shotsFired, totalShots);
            telemetry.addData("Current Slot", firingSequence != null && currentShotIndex < firingSequence.length ?
                    firingSequence[currentShotIndex] : "N/A");
            telemetry.addData("State Time", "%.1f s", stateTimer.seconds());
        }

        if (visionTargeting != null) {
            telemetry.addData("Firing Priority", visionTargeting.getFiringPriority());
            if (currentState == FireSequenceState.WAIT_FOR_VISION_LOCK ||
                    currentState == FireSequenceState.VISION_TARGETING) {
                telemetry.addData("Vision State", visionTargeting.getState());
            }
        }

        if (isInError()) {
            telemetry.addData("⚠️ ERROR", lastError);
        }
    }
}