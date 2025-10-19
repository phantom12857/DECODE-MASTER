package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DECODETeleOp", group = "Linear OpMode")
public class DECODETeleOp extends LinearOpMode {
    private DECODEMechanisms mechanisms;
    private final ElapsedTime runtime = new ElapsedTime();

    // Button state tracking
    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousY = false;
    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousBack = false;
    private boolean previousStart = false;
    private boolean previousLeftStick = false;
    private boolean previousRightStick = false;

    @Override
    public void runOpMode() {
        // Initialize mechanisms
        mechanisms = new DECODEMechanisms(hardwareMap);

        telemetry.addLine("Initialized, waiting for start...");
        telemetry.addLine("Spindexer limit switch: " +
                (mechanisms.isSpindexerLimitPressed() ? "PRESSED" : "Released"));
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Auto-home spindexer on start if limit switch is available
        if (mechanisms.isUseLimitSwitch()) {
            mechanisms.homeSpindexer();
        }

        // === Main loop ===
        while (opModeIsActive()) {
            handleDriveControls();
            handleIntakeControls();
            handleSpindexerControls();
            handleKickerControls();
            handleLauncherControls();
            handleHoodControls();
            handleTurretControls();
            handleContinuousServoControls();
            handleUtilityControls();

            // Update all mechanisms
            mechanisms.updateAllSystems();

            updateTelemetry();
        }

        mechanisms.stopAllMotors();
    }

    private void handleDriveControls() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Toggle field-centric with Y
        boolean yPressed = gamepad1.y;
        if (yPressed && !previousY) {
            mechanisms.toggleFieldCentric();
        }
        previousY = yPressed;

        // Reset heading offset with B
        if (gamepad1.b) {
            mechanisms.resetHeadingOffset();
        }

        mechanisms.driveMecanum(y, x, rx);
    }

    private void handleIntakeControls() {
        if (gamepad2.right_trigger > 0.1) {
            mechanisms.startIntake();
        } else if (gamepad2.left_trigger > 0.1) {
            mechanisms.reverseIntake();
        } else {
            mechanisms.stopIntake();
        }
    }

    private void handleSpindexerControls() {
        boolean aPressed = gamepad2.a;
        if (aPressed && !previousA && !mechanisms.isSpindexerMoving()) {
            mechanisms.advanceSpindexer();
        }

        // Manual spindexer control with X button
        boolean xPressed = gamepad2.x;
        if (xPressed && !previousLeftStick) {
            // Cycle through spindexer steps manually
            int currentStep = mechanisms.getSpindexerStep();
            mechanisms.setSpindexerStep((currentStep + 1) % 3);

        }
        previousLeftStick = xPressed;

        // Home spindexer with Y button
        boolean yPressed = gamepad2.y;
        if (yPressed && !previousY && mechanisms.isUseLimitSwitch()) {
            mechanisms.homeSpindexer();
        }
        previousY = yPressed;

        previousA = aPressed;
    }

    private void handleKickerControls() {
        boolean bPressed = gamepad2.b;
        if (bPressed && !previousB) {
            mechanisms.fireKicker();
            // Schedule retraction
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    mechanisms.retractKicker();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
        previousB = bPressed;
    }

    private void handleLauncherControls() {
        boolean rightBumper = gamepad2.right_bumper;
        boolean leftBumper = gamepad2.left_bumper;

        if (rightBumper && !previousRightBumper) {
            mechanisms.increaseLauncherRPM();
        }
        if (leftBumper && !previousLeftBumper) {
            mechanisms.decreaseLauncherRPM();
        }

        previousLeftBumper = leftBumper;
        previousRightBumper = rightBumper;

        // Stop launcher with Start button
        boolean startPressed = gamepad2.start;
        if (startPressed && !previousStart) {
            mechanisms.stopLauncher();
        }
        previousStart = startPressed;
    }

    private void handleHoodControls() {
        // Manual hood control with right stick Y
        double stick = -gamepad2.right_stick_y;
        if (Math.abs(stick) > 0.1) {
            mechanisms.adjustHoodPosition(stick * 0.02);
        }

        // Preset hood positions with D-pad
        boolean dpadUp = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;

        if (dpadUp && !previousDpadUp) {
            mechanisms.setHoodPosition(0.8); // High position
        }
        if (dpadDown && !previousDpadDown) {
            mechanisms.setHoodPosition(0.2); // Low position
        }

        previousDpadUp = dpadUp;
        previousDpadDown = dpadDown;
    }

    private void handleTurretControls() {
        // Turret control with left stick X
        double stick = gamepad2.left_stick_x;
        if (Math.abs(stick) > 0.1) {
            mechanisms.adjustTurretPosition(stick * 0.02);
        }

        // Preset turret positions with D-pad left/right
        boolean dpadLeft = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;

        if (dpadLeft && !previousDpadLeft) {
            mechanisms.setTurretPosition(0.2); // Left position
        }
        if (dpadRight && !previousDpadRight) {
            mechanisms.setTurretPosition(0.8); // Right position
        }

        // Center turret with Back button
        boolean backPressed = gamepad2.back;
        if (backPressed && !previousBack) {
            mechanisms.setTurretPosition(0.5); // Center position
        }
        previousBack = backPressed;

        previousDpadLeft = dpadLeft;
        previousDpadRight = dpadRight;
    }

    private void handleContinuousServoControls() {
        // Control continuous servo 1 with gamepad1 triggers
        if (gamepad1.right_trigger > 0.1) {
            mechanisms.setContinuousServo1Power(0.5); // Forward
        } else if (gamepad1.left_trigger > 0.1) {
            mechanisms.setContinuousServo1Power(-0.5); // Reverse
        } else {
            mechanisms.setContinuousServo1Power(0); // Stop
        }

        // Control continuous servo 2 with gamepad2 triggers
        if (gamepad2.right_trigger > 0.1) {
            mechanisms.setContinuousServo2Power(0.5); // Forward
        } else if (gamepad2.left_trigger > 0.1) {
            mechanisms.setContinuousServo2Power(-0.5); // Reverse
        } else {
            mechanisms.setContinuousServo2Power(0); // Stop
        }

        // Use left stick buttons for precise servo positioning
        boolean leftStick = gamepad1.left_stick_button;
        boolean rightStick = gamepad1.right_stick_button;

        if (leftStick && !previousLeftStick) {
            mechanisms.moveContinuousServo1Revolutions(0.25); // Move forward 0.25 revolutions
        }
        if (rightStick && !previousRightStick) {
            mechanisms.moveContinuousServo1Revolutions(-0.25); // Move backward 0.25 revolutions
        }

        previousLeftStick = leftStick;
        previousRightStick = rightStick;
    }

    private void handleUtilityControls() {
        // Toggle limit switch functionality with D-pad Up
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !previousDpadUp) {
            mechanisms.setUseLimitSwitch(!mechanisms.isUseLimitSwitch());
            telemetry.addLine("Limit switch: " +
                    (mechanisms.isUseLimitSwitch() ? "ENABLED" : "DISABLED"));
        }
        previousDpadUp = dpadUp;

        // Manual home with D-pad Down
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !previousDpadDown && mechanisms.isUseLimitSwitch()) {
            mechanisms.homeSpindexer();
        }
        previousDpadDown = dpadDown;

        // Reset all encoders with Start + Back buttons
        boolean startPressed = gamepad1.start;
        boolean backPressed = gamepad1.back;

        if (startPressed && backPressed && !previousStart) {
            mechanisms.resetAllEncoders();
            telemetry.addLine("All encoders reset!");
        }
        previousStart = startPressed;
    }

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1fs", runtime.seconds());
        telemetry.addData("Status", "All systems operational");
        mechanisms.addTelemetryData(telemetry);

        // Add control hints
        telemetry.addLine("");
        telemetry.addLine("=== Controls ===");
        telemetry.addData("Drive", "LS: Move, RS: Rotate, Y: Toggle Field-Centric, B: Reset Heading");
        telemetry.addData("Intake", "RT: Forward, LT: Reverse");
        telemetry.addData("Spindexer", "A: Advance, X: Manual Step, Y: Home, Dpad Up: Toggle Limit, Dpad Down: Re-home");
        telemetry.addData("Launcher", "RB: +RPM, LB: -RPM, Start: Stop");
        telemetry.addData("Hood", "RS: Manual, Dpad Up/Down: Presets");
        telemetry.addData("Turret", "LS: Manual, Dpad Left/Right: Presets, Back: Center");
        telemetry.addData("Servos", "Triggers: Manual, Stick Buttons: Precise");
        telemetry.addData("Reset", "Start+Back: Reset Encoders");

        telemetry.update();
    }
}