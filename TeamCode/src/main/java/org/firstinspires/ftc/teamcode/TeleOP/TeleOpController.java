// UPDATED: TeleOpController.java
package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.core.MechanismCoordinator;
import org.firstinspires.ftc.teamcode.Mechanisms.subsystems.VisionTargetingSystem;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.ButtonDebouncer;

/**
 * TeleOpController with vision-guided auto-aim capabilities.
 * <p>
 * CONTROL SCHEME:
 * <p>
 * Gamepad 2 (Operator):
 * - B = Fire single shot (auto-synced)
 * - Left Bumper = Fire 2 shots (intelligent sequencing)
 * - Right Bumper = Fire 3 shots (intelligent sequencing)
 * - A = Cancel current firing sequence
 * <p>
 * - LEFT TRIGGER (hold) = Enable vision auto-aim
 *   - While held + B = Vision-guided single shot
 *   - While held + bumpers = Vision-guided multi-shot
 * <p>
 * - X = Home spindexer
 * - Y = Reset ball count
 * - START = Toggle SYNCHRONIZED ↔ MANUAL mode
 * - BACK = Scan for monolith (sets firing priority)
 * <p>
 * - D-pad = Launcher RPM presets
 * - Right stick Y = Hood control
 * - Left stick X = Manual turret control (when not using vision)
 * <p>
 * REFACTORED: Added vision-guided auto-aim and intelligent ball sequencing
 */
public class TeleOpController {

    // ==================================================
    // D E P E N D E N C I E S
    // ==================================================
    private final DECODEMechanisms mechanisms;
    private final MechanismCoordinator coordinator;
    private final VisionTargetingSystem visionTargeting;
    private final AprilTagDetector aprilTagDetector;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Telemetry telemetry;

    // ==================================================
    // S T A T E
    // ==================================================
    private double driveSpeedModifier = 1.0;
    private boolean isBlueAlliance = true;
    private FiringMode firingMode = FiringMode.SYNCHRONIZED;
    private boolean visionAimEnabled = false;

    public enum FiringMode {
        SYNCHRONIZED,   // Auto-synced firing
        MANUAL,         // Manual control
        VISION_GUIDED   // Vision-guided with auto-aim
    }

    // ==================================================
    // B U T T O N   D E B O U N C E R S
    // ==================================================
    private final ButtonDebouncer g1_y = new ButtonDebouncer();
    private final ButtonDebouncer g1_b = new ButtonDebouncer();
    private final ButtonDebouncer g1_start = new ButtonDebouncer();

    private final ButtonDebouncer g2_a = new ButtonDebouncer();
    private final ButtonDebouncer g2_b = new ButtonDebouncer();
    private final ButtonDebouncer g2_x = new ButtonDebouncer();
    private final ButtonDebouncer g2_y = new ButtonDebouncer();
    private final ButtonDebouncer g2_back = new ButtonDebouncer();
    private final ButtonDebouncer g2_left_bumper = new ButtonDebouncer();
    private final ButtonDebouncer g2_right_bumper = new ButtonDebouncer();
    private final ButtonDebouncer g2_dpad_up = new ButtonDebouncer();
    private final ButtonDebouncer g2_dpad_down = new ButtonDebouncer();
    private final ButtonDebouncer g2_dpad_left = new ButtonDebouncer();
    private final ButtonDebouncer g2_dpad_right = new ButtonDebouncer();
    private final ButtonDebouncer g2_start = new ButtonDebouncer();

    /**
     * Constructor for TeleOpController with vision.
     */
    public TeleOpController(DECODEMechanisms mechanisms,
                            AprilTagDetector aprilTagDetector,
                            Gamepad gamepad1,
                            Gamepad gamepad2,
                            Telemetry telemetry) {
        this.mechanisms = mechanisms;
        this.aprilTagDetector = aprilTagDetector;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        // Initialize vision targeting if turret is available
        if (mechanisms.hasTurret()) {
            this.visionTargeting = new VisionTargetingSystem(mechanisms.hardwareMap, mechanisms.turret);
        } else {
            this.visionTargeting = null;
        }

        // Create coordinator with vision integration
        if (mechanisms.hasSpindexer() && mechanisms.hasLauncher()) {
            this.coordinator = new MechanismCoordinator(
                    mechanisms.spindexer,
                    mechanisms.launcher,
                    visionTargeting
            );
        } else {
            this.coordinator = null;
            if (telemetry != null) {
                telemetry.addData("Warning", "MechanismCoordinator unavailable");
            }
        }
    }

    /**
     * Main update loop with exception handling.
     */
    public void update() {
        try {
            updateButtonDebouncers();
            handleEmergencyStop();
            handleDriveControls();
            handleIntakeControls();
            handleLauncherControls();
            handleSpindexerControls();
            handleHoodAndTurretControls();
            handleVisionControls();
            handleCoordinatorControls();

            if (coordinator != null) {
                coordinator.update();
            }

            // Update vision targeting if available (only when enabled)
            if (visionTargeting != null) {
                visionTargeting.update();
            }
        } catch (Exception e) {
            // Log error but don't crash - robot continues functioning
            if (telemetry != null) {
                telemetry.addData("⚠️ Controller Error", e.getClass().getSimpleName());
                telemetry.addData("Message", e.getMessage());
            }
        }
    }

    private void updateButtonDebouncers() {
        // Gamepad 1
        g1_y.update(gamepad1.y);
        g1_b.update(gamepad1.b);
        g1_start.update(gamepad1.start);

        // Gamepad 2
        g2_a.update(gamepad2.a);
        g2_b.update(gamepad2.b);
        g2_x.update(gamepad2.x);
        g2_y.update(gamepad2.y);
        g2_back.update(gamepad2.back);
        g2_left_bumper.update(gamepad2.left_bumper);
        g2_right_bumper.update(gamepad2.right_bumper);
        g2_dpad_up.update(gamepad2.dpad_up);
        g2_dpad_down.update(gamepad2.dpad_down);
        g2_dpad_left.update(gamepad2.dpad_left);
        g2_dpad_right.update(gamepad2.dpad_right);
        g2_start.update(gamepad2.start);
    }

    // ==================================================
    // C O N T R O L   H A N D L E R S
    // ==================================================

    private void handleEmergencyStop() {
        if (gamepad1.start && gamepad2.start) {
            mechanisms.emergencyStop();
            if (coordinator != null) coordinator.cancel();
            telemetry.addLine("🚨 EMERGENCY STOP 🚨");
        }
    }

    private void handleDriveControls() {
        if (!mechanisms.hasDrive()) return;

        driveSpeedModifier = 1.0 - (gamepad1.right_trigger * 0.7);

        double y = -gamepad1.left_stick_y * driveSpeedModifier;
        double x = gamepad1.left_stick_x * driveSpeedModifier;
        double rx = gamepad1.right_stick_x * driveSpeedModifier;

        if (g1_y.wasPressedThisCycle()) {
            mechanisms.drive.toggleFieldCentric();
        }
        if (g1_b.wasPressedThisCycle()) {
            mechanisms.drive.resetHeadingOffset();
        }

        mechanisms.drive.driveMecanum(y, x, rx);
    }

    private void handleIntakeControls() {
        if (!mechanisms.hasIntake()) return;

        if (gamepad2.right_trigger > 0.1 && gamepad2.left_trigger < 0.1) {
            // Right trigger only = full intake
            mechanisms.intake.start();
        } else if (gamepad2.left_trigger > 0.5) {
            // Left trigger at least halfway = vision aim mode
            visionAimEnabled = true;
            mechanisms.intake.stop();
        } else {
            // Neither trigger pressed = passive intake to maintain grip
            visionAimEnabled = false;
            mechanisms.intake.passiveIntake();
        }
    }

    private void handleLauncherControls() {
        if (!mechanisms.hasLauncher()) return;

        // D-Pad RPM presets
        if (g2_dpad_up.wasPressedThisCycle()) mechanisms.launcher.setRPM(4500);
        if (g2_dpad_left.wasPressedThisCycle()) mechanisms.launcher.setRPM(3750);
        if (g2_dpad_right.wasPressedThisCycle()) mechanisms.launcher.setRPM(3250);
        if (g2_dpad_down.wasPressedThisCycle()) mechanisms.launcher.stop();

        // Manual fire (only in MANUAL mode)
        if (firingMode == FiringMode.MANUAL) {
            if (g2_b.wasPressedThisCycle()) {
                mechanisms.launcher.kick();
            }
        }
    }

    private void handleSpindexerControls() {
        if (!mechanisms.hasSpindexer()) return;

        // Manual advance (only in MANUAL mode)
        if (firingMode == FiringMode.MANUAL) {
            if (g2_a.wasPressedThisCycle()) {
                mechanisms.spindexer.increment();
            }
        }

        if (g2_x.wasPressedThisCycle()) {
            mechanisms.spindexer.home();
        }

        if (g2_y.wasPressedThisCycle()) {
            mechanisms.spindexer.setBallsLoaded(0);
        }
    }

    private void handleVisionControls() {
        if (visionTargeting == null) return;

        // Enable vision when left trigger is held (for auto-aim)
        boolean visionRequested = gamepad2.left_trigger > 0.5;
        visionTargeting.setEnabled(visionRequested);

        // Update vision aim state
        visionAimEnabled = visionRequested && visionTargeting.isLocked();

        // BACK button = scan for monolith
        if (g2_back.wasPressedThisCycle()) {
            if (coordinator != null) {
                boolean found = coordinator.scanForFiringPriority();
                if (found) {
                    gamepad2.rumble(200);  // Success feedback
                    telemetry.addLine("✓ Monolith scanned!");
                } else {
                    gamepad2.rumble(100, 100, 50);  // Not found pattern
                    telemetry.addLine("⚠️ Monolith not found");
                }
                telemetry.update();
            }
        }
    }

    private void handleCoordinatorControls() {
        if (coordinator == null) return;

        // START = Toggle mode
        if (g2_start.wasPressedThisCycle()) {
            firingMode = (firingMode == FiringMode.SYNCHRONIZED) ?
                    FiringMode.MANUAL : FiringMode.SYNCHRONIZED;
            telemetry.addData("Mode", firingMode);
            telemetry.update();
        }

        // Only allow firing in SYNCHRONIZED mode
        if (firingMode == FiringMode.SYNCHRONIZED) {
            // A = Cancel
            if (g2_a.wasPressedThisCycle()) {
                coordinator.cancel();
            }

            // Check if vision aim is enabled
            boolean useVision = visionAimEnabled && visionTargeting != null;

            // B = Single shot
            if (g2_b.wasPressedThisCycle()) {
                boolean started;
                if (useVision) {
                    started = coordinator.startVisionGuidedFire(1);
                } else {
                    started = coordinator.fireSingleShot();
                }

                if (!started) {
                    gamepad2.rumble(100);  // Busy feedback
                }
            }

            // Left bumper = 2 shots
            if (g2_left_bumper.wasPressedThisCycle()) {
                boolean started;
                if (useVision) {
                    started = coordinator.startVisionGuidedFire(2);
                } else {
                    started = coordinator.startMultiShotSequence(2);
                }

                if (!started) {
                    gamepad2.rumble(100);
                }
            }

            // Right bumper = 3 shots
            if (g2_right_bumper.wasPressedThisCycle()) {
                boolean started;
                if (useVision) {
                    started = coordinator.startVisionGuidedFire(3);
                } else {
                    started = coordinator.startMultiShotSequence(3);
                }

                if (!started) {
                    gamepad2.rumble(100);
                }
            }
        }
    }

    private void handleHoodAndTurretControls() {
        if (mechanisms.hasHood()) {
            mechanisms.hood.setPower(-gamepad2.right_stick_y);
        }

        // Only allow manual turret control when not using vision
        if (mechanisms.hasTurret() && !visionAimEnabled) {
            mechanisms.turret.setPower(gamepad2.left_stick_x);
        }
    }

    /**
     * Adds control telemetry.
     */
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addLine("=== Controls ===");
        telemetry.addData("Drive Speed", "%.0f%%", driveSpeedModifier * 100);
        telemetry.addData("Alliance", isBlueAlliance ? "Blue" : "Red");
        telemetry.addData("Mode", firingMode);

        if (visionAimEnabled) {
            telemetry.addData("Vision Aim", "🎯 ACTIVE");
        }

        if (coordinator != null && firingMode == FiringMode.SYNCHRONIZED) {
            telemetry.addLine();
            coordinator.addTelemetryData(telemetry);
        }
    }

    public void setAlliance(boolean isBlue) {
        this.isBlueAlliance = isBlue;
        if (visionTargeting != null) {
            visionTargeting.setAlliance(isBlue);
        }
    }

    public FiringMode getFiringMode() {
        return firingMode;
    }

    public MechanismCoordinator getCoordinator() {
        return coordinator;
    }
}