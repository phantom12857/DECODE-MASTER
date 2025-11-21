package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.ButtonDebouncer;

/**
 * The TeleOpController class centralizes all driver control logic.
 * It maps gamepad inputs to specific robot actions and provides a clear separation
 * between the control scheme and the underlying mechanism implementation.
 */
public class TeleOpController {

    // ==================================================
    // D E P E N D E N C I E S
    // ==================================================
    private final DECODEMechanisms mechanisms;
    private final AprilTagDetector aprilTagDetector;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Telemetry telemetry;

    // ==================================================
    // S T A T E
    // ==================================================
    private double driveSpeedModifier = 1.0;
    private boolean isBlueAlliance = true; // Default, can be changed.

    // ==================================================
    // B U T T O N   D E B O U N C E R S
    // ==================================================
    private final ButtonDebouncer g1_y = new ButtonDebouncer();
    private final ButtonDebouncer g1_b = new ButtonDebouncer();
    private final ButtonDebouncer g2_a = new ButtonDebouncer();
    private final ButtonDebouncer g2_b = new ButtonDebouncer();
    private final ButtonDebouncer g2_x = new ButtonDebouncer();
    private final ButtonDebouncer g2_y = new ButtonDebouncer();
    private final ButtonDebouncer g2_dpad_up = new ButtonDebouncer();
    private final ButtonDebouncer g2_dpad_down = new ButtonDebouncer();
    private final ButtonDebouncer g2_dpad_left = new ButtonDebouncer();
    private final ButtonDebouncer g2_dpad_right = new ButtonDebouncer();

    /**
     * Constructor for TeleOpController.
     */
    public TeleOpController(DECODEMechanisms mechanisms, AprilTagDetector aprilTagDetector,
                            Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.mechanisms = mechanisms;
        this.aprilTagDetector = aprilTagDetector;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    /**
     * Main update loop for the controller.
     */
    public void update() {
        updateButtonDebouncers();
        handleDriveControls();
        handleIntakeControls();
        handleLauncherControls();
        handleSpindexerControls();
        handleHoodAndTurretControls();
    }

    /**
     * Updates the state of all debouncers.
     */
    private void updateButtonDebouncers() {
        g1_y.update(gamepad1.y);
        g1_b.update(gamepad1.b);
        g2_a.update(gamepad2.a);
        g2_b.update(gamepad2.b);
        g2_x.update(gamepad2.x);
        g2_y.update(gamepad2.y);
        g2_dpad_up.update(gamepad2.dpad_up);
        g2_dpad_down.update(gamepad2.dpad_down);
        g2_dpad_left.update(gamepad2.dpad_left);
        g2_dpad_right.update(gamepad2.dpad_right);
    }

    // ==================================================
    // C O N T R O L   H A N D L E R S
    // ==================================================

    private void handleDriveControls() {
        // Right trigger acts as a variable speed modifier (the more you press, the slower you go).
        driveSpeedModifier = 1.0 - (gamepad1.right_trigger * 0.7);

        double y = -gamepad1.left_stick_y * driveSpeedModifier;
        double x = gamepad1.left_stick_x * driveSpeedModifier;
        double rx = gamepad1.right_stick_x * driveSpeedModifier;

        // Field-centric and heading reset controls.
        if (g1_y.wasPressedThisCycle()) mechanisms.drive.toggleFieldCentric();
        if (g1_b.wasPressedThisCycle()) mechanisms.drive.resetHeadingOffset();

        mechanisms.drive.driveMecanum(y, x, rx);
    }

    private void handleIntakeControls() {
        if (gamepad2.right_trigger > 0.1) {
            mechanisms.intake.start();
        } else if (gamepad2.left_trigger > 0.1) {
            mechanisms.intake.reverse();
        } else {
            mechanisms.intake.stop();
        }
    }

    private void handleLauncherControls() {
        // D-Pad for RPM presets.
        if (g2_dpad_up.wasPressedThisCycle()) mechanisms.launcher.setRPM(4500);
        if (g2_dpad_left.wasPressedThisCycle()) mechanisms.launcher.setRPM(3750);
        if (g2_dpad_right.wasPressedThisCycle()) mechanisms.launcher.setRPM(3250);
        if (g2_dpad_down.wasPressedThisCycle()) mechanisms.launcher.stop();

        // B button to fire.
        if (g2_b.wasPressedThisCycle()) mechanisms.launcher.kick();
    }

    private void handleSpindexerControls() {
        // A to advance, X to home, Y to reset ball count.
        if (g2_a.wasPressedThisCycle()) mechanisms.spindexer.increment();
        if (g2_x.wasPressedThisCycle()) mechanisms.spindexer.home();
        if (g2_y.wasPressedThisCycle()) mechanisms.spindexer.setBallsLoaded(0);
    }

    private void handleHoodAndTurretControls() {
        // Right stick for hood, Left stick for turret.
        mechanisms.hood.setPower(-gamepad2.right_stick_y);
        mechanisms.turret.setPower(gamepad2.left_stick_x);
    }

    /**
     * Adds control-specific data to the telemetry.
     */
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addLine("--- Driver Controls ---");
        telemetry.addData("Drive Speed Modifier", "%.2f", driveSpeedModifier);
        telemetry.addData("Alliance", isBlueAlliance ? "Blue" : "Red");
    }

    /**
     * Sets the alliance for AprilTag detection.
     */
    public void setAlliance(boolean isBlue) {
        this.isBlueAlliance = isBlue;
    }
}
