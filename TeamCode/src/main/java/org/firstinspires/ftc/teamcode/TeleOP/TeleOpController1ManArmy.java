package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.ButtonDebouncer;

/**
 * The TeleOpController1ManArmy class provides an alternative control scheme
 * where a single driver can control all essential robot functions from one gamepad.
 */
public class  TeleOpController1ManArmy {

    // ==================================================
    // D E P E N D E N C I E S
    // ==================================================
    private final DECODEMechanisms mechanisms;
    private final AprilTagDetector aprilTagDetector;
    private final Gamepad gamepad1;
    private final Telemetry telemetry;

    // ==================================================
    // S T A T E
    // ==================================================
    private double driveSpeedModifier = 1.0;
    private boolean isBlueAlliance = true;

    // ==================================================
    // B U T T O N   D E B O U N C E R S
    // ==================================================
    private final ButtonDebouncer g1_y = new ButtonDebouncer();
    private final ButtonDebouncer g1_b = new ButtonDebouncer();
    private final ButtonDebouncer g1_a = new ButtonDebouncer();
    private final ButtonDebouncer g1_x = new ButtonDebouncer();
    private final ButtonDebouncer g1_lb = new ButtonDebouncer();
    private final ButtonDebouncer g1_rb = new ButtonDebouncer();
    private final ButtonDebouncer g1_dpad_up = new ButtonDebouncer();
    private final ButtonDebouncer g1_dpad_down = new ButtonDebouncer();
    private final ButtonDebouncer g1_dpad_left = new ButtonDebouncer();
    private final ButtonDebouncer g1_dpad_right = new ButtonDebouncer();

    /**
     * Constructor for TeleOpController1ManArmy.
     */
    public TeleOpController1ManArmy(DECODEMechanisms mechanisms, AprilTagDetector aprilTagDetector,
                                    Gamepad gamepad1, Telemetry telemetry) {
        this.mechanisms = mechanisms;
        this.aprilTagDetector = aprilTagDetector;
        this.gamepad1 = gamepad1; // This controller now correctly uses only gamepad1.
        this.telemetry = telemetry;
    }

    /**
     * Main update loop for the controller.
     */
    public void update() {
        updateButtonDebouncers();
        handleDriveControls();
        handleIntakeControls();
        handleLauncherAndSpindexerControls();
    }

    /**
     * Updates the state of all debouncers.
     */
    private void updateButtonDebouncers() {
        g1_y.update(gamepad1.y);
        g1_b.update(gamepad1.b);
        g1_a.update(gamepad1.a);
        g1_x.update(gamepad1.x);
        g1_lb.update(gamepad1.left_bumper);
        g1_rb.update(gamepad1.right_bumper);
        g1_dpad_up.update(gamepad1.dpad_up);
        g1_dpad_down.update(gamepad1.dpad_down);
        g1_dpad_left.update(gamepad1.dpad_left);
        g1_dpad_right.update(gamepad1.dpad_right);
    }

    // ==================================================
    // C O N T R O L   H A N D L E R S
    // ==================================================

    private void handleDriveControls() {
        driveSpeedModifier = 1.0 - (gamepad1.right_trigger * 0.7);
        double y = -gamepad1.left_stick_y * driveSpeedModifier;
        double x = gamepad1.left_stick_x * driveSpeedModifier;
        double rx = gamepad1.right_stick_x * driveSpeedModifier;
        if (g1_y.wasPressedThisCycle()) mechanisms.drive.toggleFieldCentric();
        mechanisms.drive.driveMecanum(y, x, rx);
    }

    private void handleIntakeControls() {
        if (g1_rb.wasPressedThisCycle()) {
            if (mechanisms.intake.getState() == IntakeSystem.State.INTAKING) {
                mechanisms.intake.stop();
            } else {
                mechanisms.intake.start();
            }
        } else if (g1_lb.wasPressedThisCycle()) {
            if (mechanisms.intake.getState() == IntakeSystem.State.REVERSING) {
                mechanisms.intake.stop();
            } else {
                mechanisms.intake.reverse();
            }
        }
    }

    private void handleLauncherAndSpindexerControls() {
        // D-Pad for RPM presets
        if (g1_dpad_up.wasPressedThisCycle()) mechanisms.launcher.setRPM(4500);
        if (g1_dpad_left.wasPressedThisCycle()) mechanisms.launcher.setRPM(3750);
        if (g1_dpad_right.wasPressedThisCycle()) mechanisms.launcher.setRPM(3250);
        if (g1_dpad_down.wasPressedThisCycle()) mechanisms.launcher.stop();

        // X to fire, which also clears the spindexer slot
        if (g1_x.wasPressedThisCycle()) {
            mechanisms.launcher.kick();
            mechanisms.spindexer.clearSlot(mechanisms.spindexer.getCurrentStep());
        }

        // A to manually advance the spindexer
        if (g1_a.wasPressedThisCycle()) {
            mechanisms.spindexer.increment();
        }
        
        // B to toggle the auto-indexing feature
        if (g1_b.wasPressedThisCycle()) {
            mechanisms.spindexer.toggleAutoIndex();
        }
    }

    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addLine("--- 1-Man Army Controls ---");
        telemetry.addData("Drive Speed Modifier", "%.2f", driveSpeedModifier);
    }

    public void setAlliance(boolean isBlue) {
        this.isBlueAlliance = isBlue;
    }
}
