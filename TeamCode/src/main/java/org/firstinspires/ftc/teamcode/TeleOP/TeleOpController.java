package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.ButtonDebouncer;

public class TeleOpController {
    private final DECODEMechanisms mechanisms;
    private final AprilTagDetector aprilTagDetector;
    private final Gamepad gamepad1, gamepad2;
    private final Telemetry telemetry;

    private double driveSpeedModifier = 1.0;
    private double desiredRPM = 0.0;
    private final double REGRESSION_SLOPE = 13.98485;
    private final double REGRESSION_Y_INT = 2834.57576;

    // Button debouncers for each button
    private final ButtonDebouncer g1RightBumper = new ButtonDebouncer();
    private final ButtonDebouncer g1LeftBumper = new ButtonDebouncer();
    private final ButtonDebouncer g1Y = new ButtonDebouncer();
    private final ButtonDebouncer g2A = new ButtonDebouncer();
    private final ButtonDebouncer g2X = new ButtonDebouncer();
    private final ButtonDebouncer g2Y = new ButtonDebouncer();
    private final ButtonDebouncer g2B = new ButtonDebouncer();
    private final ButtonDebouncer g2LeftBumper = new ButtonDebouncer();
    private final ButtonDebouncer g2DpadLeft = new ButtonDebouncer();
    private final ButtonDebouncer g2DpadUp = new ButtonDebouncer();
    private final ButtonDebouncer g2DpadRight = new ButtonDebouncer();
    private final ButtonDebouncer g2DpadDown = new ButtonDebouncer();
    private final ButtonDebouncer g1DpadDown = new ButtonDebouncer();
    private final ButtonDebouncer g1LeftStick = new ButtonDebouncer();
    private final ButtonDebouncer g1RightStick = new ButtonDebouncer();

    public TeleOpController(DECODEMechanisms mechanisms, AprilTagDetector aprilTagDetector,
                            Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.mechanisms = mechanisms;
        this.aprilTagDetector = aprilTagDetector;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void update() {
        // Update all button debouncers
        updateButtonDebouncers();

        handleDriveControls();
        handleIntakeControls();
        handleSpindexerControls();
        handleKickerControls();
        handleLauncherControls();
        handleHoodControls();
        handleTurretControls();
        handleContinuousServoControls();
        handleUtilityControls();
    }

    private void updateButtonDebouncers() {
        g1RightBumper.update(gamepad1.right_bumper);
        g1LeftBumper.update(gamepad1.left_bumper);
        g1Y.update(gamepad1.y);
        g2A.update(gamepad2.a);
        g2X.update(gamepad2.x);
        g2Y.update(gamepad2.y);
        g2B.update(gamepad2.b);
        g2LeftBumper.update(gamepad2.left_bumper);
        g2DpadLeft.update(gamepad2.dpad_left);
        g2DpadUp.update(gamepad2.dpad_up);
        g2DpadRight.update(gamepad2.dpad_right);
        g2DpadDown.update(gamepad2.dpad_down);
        g1DpadDown.update(gamepad1.dpad_down);
        g1LeftStick.update(gamepad1.left_stick_button);
        g1RightStick.update(gamepad1.right_stick_button);
    }

    private void handleDriveControls() {
        // Speed modifier controls with debouncing
        if (g1RightBumper.wasPressedThisCycle()) {
            driveSpeedModifier = Math.min(1.0, driveSpeedModifier + 0.1);
        }

        if (g1LeftBumper.wasPressedThisCycle()) {
            driveSpeedModifier = Math.max(0.1, driveSpeedModifier - 0.1);
        }

        double y = -gamepad1.left_stick_y * driveSpeedModifier;
        double x = gamepad1.left_stick_x * driveSpeedModifier;
        double rx = gamepad1.right_stick_x * driveSpeedModifier;

        // Toggle field-centric with debouncing
        if (g1Y.wasPressedThisCycle()) {
            mechanisms.drive.toggleFieldCentric();
        }

        // Reset heading offset
        if (gamepad1.b) {
            mechanisms.drive.resetHeadingOffset();
        }

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

    private void handleSpindexerControls() {
        // Manual advance with debouncing
        if (g2A.wasPressedThisCycle() && !mechanisms.spindexer.isMoving()) {
            mechanisms.spindexer.increment();
        }

        // Manual step control with debouncing
        if (g2X.wasPressedThisCycle() && !mechanisms.spindexer.isMoving()) {
            int currentStep = mechanisms.spindexer.getCurrentStep();
            int nextStep = (currentStep + 1) % 3;
            mechanisms.spindexer.moveToStep(nextStep);
        }

        // Home spindexer with debouncing
        if (g2Y.wasPressedThisCycle()) {
            mechanisms.spindexer.home();
        }
    }

    private void handleKickerControls() {
        if (g2B.wasPressedThisCycle()) {
            mechanisms.launcher.kick();
        }
    }

    private void handleLauncherControls() {
        // Preset RPM controls with debouncing
        if (g2DpadLeft.wasPressedThisCycle()) {
            mechanisms.launcher.setRPM(3250);
        }

        if (g2DpadUp.wasPressedThisCycle()) {
            mechanisms.launcher.setRPM(3750);
        }

        if (g2DpadRight.wasPressedThisCycle()) {
            mechanisms.launcher.setRPM(4500);
        }

        // Auto RPM based on AprilTag distance
        double distanceToApriltag = aprilTagDetector.getDistance();
        desiredRPM = distanceToApriltag * REGRESSION_SLOPE + REGRESSION_Y_INT;

        if (g2LeftBumper.wasPressedThisCycle()) {
            mechanisms.launcher.setRPM(desiredRPM);
        }

        // Stop launcher with debouncing
        if (g2DpadDown.wasPressedThisCycle()) {
            mechanisms.launcher.stop();
        }
    }

    private void handleHoodControls() {
        // Manual hood control
        double stick = -gamepad2.right_stick_y;
        if (Math.abs(stick) > 0.1) {
            mechanisms.hood.setPower(stick * 0.8);
        } else {
            mechanisms.hood.stop();
        }

        // Preset hood movements with debouncing
        if (g2DpadUp.wasPressedThisCycle()) {
            mechanisms.hood.setPower(0.6);
        }
        if (g2DpadDown.wasPressedThisCycle()) {
            mechanisms.hood.setPower(-0.6);
        }

        // Stop when buttons released
        if (g2DpadUp.wasReleased()) {
            mechanisms.hood.stop();
        }
        if (g2DpadDown.wasReleased()) {
            mechanisms.hood.stop();
        }
    }

    private void handleTurretControls() {
        // Manual turret control
        double stick = gamepad2.left_stick_x;
        if (Math.abs(stick) > 0.1) {
            mechanisms.turret.setPower(stick * 0.8);
        } else {
            mechanisms.turret.stop();
        }

        // Preset turret movements with debouncing
        if (g2DpadLeft.wasPressedThisCycle()) {
            mechanisms.turret.setPower(-0.6);
        }
        if (g2DpadRight.wasPressedThisCycle()) {
            mechanisms.turret.setPower(0.6);
        }

        // Stop when buttons released
        if (g2DpadLeft.wasReleased()) {
            mechanisms.turret.stop();
        }
        if (g2DpadRight.wasReleased()) {
            mechanisms.turret.stop();
        }
    }

    private void handleContinuousServoControls() {
        // Continuous servo 1 with triggers
        if (gamepad1.right_trigger > 0.1) {
            mechanisms.continuousServos.setPosition(1, 0.5);
        } else if (gamepad1.left_trigger > 0.1) {
            mechanisms.continuousServos.setPosition(1, -0.5);
        } else {
            mechanisms.continuousServos.setPosition(1, 0);
        }

        // Continuous servo 2 with triggers
        if (gamepad2.right_trigger > 0.1) {
            mechanisms.continuousServos.setPosition(2, 0.5);
        } else if (gamepad2.left_trigger > 0.1) {
            mechanisms.continuousServos.setPosition(2, -0.5);
        } else {
            mechanisms.continuousServos.setPosition(2, 0);
        }

        // Precise positioning with debouncing
        if (g1LeftStick.wasPressedThisCycle()) {
            double currentPos = mechanisms.continuousServos.getPositionRevolutions(1);
            mechanisms.continuousServos.setPosition(1, currentPos + 0.25);
        }

        if (g1RightStick.wasPressedThisCycle()) {
            double currentPos = mechanisms.continuousServos.getPositionRevolutions(1);
            mechanisms.continuousServos.setPosition(1, currentPos - 0.25);
        }
    }

    private void handleUtilityControls() {
        // Manual home spindexer with debouncing
        if (g1DpadDown.wasPressedThisCycle()) {
            mechanisms.spindexer.home();
        }
    }

    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Drive Speed", "%.1fx", driveSpeedModifier);
        telemetry.addData("Auto RPM Target", "%.0f", desiredRPM);
        telemetry.addLine("");
        telemetry.addLine("=== Controls ===");
        telemetry.addData("Drive", "LS: Move, RS: Rotate, Y: Toggle Field-Centric");
        telemetry.addData("Intake", "RT: Forward, LT: Reverse");
        telemetry.addData("Spindexer", "A: Manual Advance, X: Manual Step, Y: Home");
        telemetry.addData("Launcher", "Dpad: Presets, LB: Auto RPM, Start: Stop");
        telemetry.addData("Hood", "RS: Manual, Dpad Up/Down: Move");
        telemetry.addData("Turret", "LS: Manual, Dpad Left/Right: Move");

        // Debug button states
        telemetry.addLine("");
        telemetry.addLine("=== Button Debug ===");
        telemetry.addData("G2 A Pressed", g2A.wasPressedThisCycle());
        telemetry.addData("G2 B Pressed", g2B.wasPressedThisCycle());
        telemetry.addData("G2 X Pressed", g2X.wasPressedThisCycle());
        telemetry.addData("G2 Y Pressed", g2Y.wasPressedThisCycle());
    }
}