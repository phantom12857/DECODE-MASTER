/*
================================================================================================
CONTROL MAP
================================================================================================

--- Gamepad 1 (Driver) ---
Left Stick Y:        Drive robot forward/backward
Left Stick X:        Strafe robot left/right
Right Stick X:       Rotate robot left/right

Left Bumper:         Decrease drive speed modifier by 0.1 (min 0.1)
Right Bumper:        Increase drive speed modifier by 0.1 (max 1.0)

Y Button:            Toggle between field-centric and robot-centric driving
B Button:            Reset IMU heading offset (for field-centric driving)

D-Pad Up:            Toggle spindexer limit switch functionality
D-Pad Down:          Manually home the spindexer (if limit switch is enabled)

Left Stick Button:   Move continuous servo 1 forward by 0.25 revolutions
Right Stick Button:  Move continuous servo 1 backward by 0.25 revolutions

Right Trigger:       Run continuous servo 1 forward
Left Trigger:        Run continuous servo 1 backward

Start + Back:        Reset all motor encoders

--- Gamepad 2 (Operator) ---
Right Trigger:       Start intake
Left Trigger:        Reverse intake

A Button:            Manually advance spindexer to next position (overrides auto)
B Button:            Fire kicker to launch ring
X Button:            Manually cycle spindexer to the next step
Y Button:            Home the spindexer

Right Bumper:        Increase launcher RPM
Left Bumper:         Decrease launcher RPM
Start Button:        Stop the launcher motor

Right Stick Y:       Manually adjust hood position (power control for CRServo)
D-Pad Up:            Set hood to move up
D-Pad Down:          Set hood to move down

Left Stick X:        Manually adjust turret position (power control for CRServo)
D-Pad Right:         Set turret to move right
D-Pad Left:          Set turret to move left
Back Button:         Stop turret

Right Trigger:       Run continuous servo 2 forward
Left Trigger:        Run continuous servo 2 backward

--- Servo Testing (Gamepad1) ---
X Button:            Test all servos
A Button:            Set servos to test positions
D-Pad:               Manual servo control

--- Spindexer Auto-Intake ---
AUTO BY DEFAULT:     Automatically detects balls and rotates to next slot
A Button (G2):       Manual override - advance to next position
Y Button (G2):       Manual home position

--- Hood PID Holding ---
AUTO:                Automatically holds position when not manually moving
                    Prevents vibration-induced movement

================================================================================================
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DECODETeleOp", group = "Linear OpMode")
public class DECODETeleOp extends LinearOpMode {
    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;
    private final ElapsedTime runtime = new ElapsedTime();
    private double driveSpeedModifier = 1.0;

    private double desiredRPM = 0.0;
    private final double REGRESSION_SLOPE = 13.98485;
    private final double REGRESSION_Y_INT = 2834.57576;
    private double distanceToApriltag = 0.0;

    // Button state tracking
    // Gamepad 1
    private boolean previousG1_Y = false;
    private boolean previousG1_LeftBumper = false;
    private boolean previousG1_RightBumper = false;
    private boolean previousG1_DpadUp = false;
    private boolean previousG1_DpadDown = false;
    private boolean previousG1_Back = false;
    private boolean previousG1_Start = false;
    private boolean previousG1_LeftStick = false;
    private boolean previousG1_RightStick = false;
    private boolean previousG1_X = false;
    private boolean previousG1_A = false;

    // Gamepad 2
    private boolean previousG2_A = false;
    private boolean previousG2_B = false;
    private boolean previousG2_X = false;
    private boolean previousG2_Y = false;
    private boolean previousG2_LeftBumper = false;
    private boolean previousG2_RightBumper = false;
    private boolean previousNear = false;
    private boolean previousMid = false;
    private boolean previousFar = false;
    private boolean previousG2_DpadLeft = false;
    private boolean previousG2_DpadRight = false;
    private boolean previousG2_Back = false;
    private boolean previousOff = false;

    @Override
    public void runOpMode() {
        // Initialize mechanisms
        mechanisms = new DECODEMechanisms(hardwareMap);
        aprilTagDetector = new AprilTagDetector(hardwareMap);

        telemetry.addLine("Initialized, waiting for start...");
        telemetry.addLine("Spindexer limit switch: " +
                (mechanisms.isSpindexerLimitPressed() ? "PRESSED" : "Released"));
        telemetry.addLine("Auto-Intake: ENABLED by default");
        telemetry.addLine("Hood PID Holding: ACTIVE");
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
            handleServoTestControls();

            // Update all mechanisms
            mechanisms.updateAllSystems();

            updateTelemetry();
        }

        mechanisms.stopAllMotors();
    }

    private void handleDriveControls() {
        boolean rightBumperPressed = gamepad1.right_bumper;
        if (rightBumperPressed && !previousG1_RightBumper) {
            driveSpeedModifier = Math.min(1.0, driveSpeedModifier + 0.1);
        }
        previousG1_RightBumper = rightBumperPressed;

        boolean leftBumperPressed = gamepad1.left_bumper;
        if (leftBumperPressed && !previousG1_LeftBumper) {
            driveSpeedModifier = Math.max(0.1, driveSpeedModifier - 0.1);
        }
        previousG1_LeftBumper = leftBumperPressed;

        double y = -gamepad1.left_stick_y * driveSpeedModifier;
        double x = gamepad1.left_stick_x * driveSpeedModifier;
        double rx = gamepad1.right_stick_x * driveSpeedModifier;

        // Toggle field-centric with Y
        boolean yPressed = gamepad1.y;
        if (yPressed && !previousG1_Y) {
            mechanisms.toggleFieldCentric();
        }
        previousG1_Y = yPressed;

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
        // Manual advance with A button (overrides auto)
        boolean aPressed = gamepad2.a;
        if (aPressed && !previousG2_A && !mechanisms.isSpindexerMoving()) {
            mechanisms.manualAdvanceSpindexer();
            telemetry.addLine("Manual advance - Auto-Intake temporarily overridden");
        }
        previousG2_A = aPressed;

        // Manual spindexer control with X button
        boolean xPressed = gamepad2.x;
        if (xPressed && !previousG2_X && !mechanisms.isSpindexerMoving()) {
            int currentStep = mechanisms.getSpindexerStep();
            int nextStep = (currentStep + 1) % 3;
            mechanisms.setSpindexerStep(nextStep);
            telemetry.addLine("Manual step set to: " + nextStep);
        }
        previousG2_X = xPressed;

        // Home spindexer with Y button
        boolean yPressed = gamepad2.y;
        if (yPressed && !previousG2_Y) {
            mechanisms.homeSpindexer();
            telemetry.addLine("Spindexer homed");
        }
        previousG2_Y = yPressed;
    }

    private void handleKickerControls() {
        boolean bPressed = gamepad2.b;
        if (bPressed && !previousG2_B) {
            mechanisms.fireKicker();
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    mechanisms.retractKicker();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
        previousG2_B = bPressed;
    }

    private void handleLauncherControls() {
        boolean rightBumper = gamepad2.right_bumper;
        boolean leftBumper = gamepad2.left_bumper;

        boolean near = gamepad2.dpad_left;
        boolean mid = gamepad2.dpad_up;
        boolean far = gamepad2.dpad_right;
        boolean off = gamepad2.dpad_down;

//        if (rightBumper && !previousG2_RightBumper) {
//            mechanisms.increaseLauncherRPM();
//        }
//        if (leftBumper && !previousG2_LeftBumper) {
//            mechanisms.decreaseLauncherRPM();
//        }
        if (near && !previousNear) {
            mechanisms.setLauncherRPM(3250);
        } if (mid && !previousMid) {
            mechanisms.setLauncherRPM(3750);
        } if (far && !previousFar) {
            mechanisms.setLauncherRPM(4500);
        }

        distanceToApriltag = aprilTagDetector.getDistance();
        desiredRPM = distanceToApriltag * REGRESSION_SLOPE + REGRESSION_Y_INT;

        if (leftBumper && !previousG2_LeftBumper) {
            mechanisms.setLauncherRPM(desiredRPM);
        }

        previousG2_LeftBumper = leftBumper;
        previousG2_RightBumper = rightBumper;

        // Stop launcher with dpad-down button
        if (off && !previousOff) {
            mechanisms.stopLauncher();
        }
        previousOff = off;
    }

    private void handleHoodControls() {
        // Manual hood control with right stick Y - direct power control
//        double stick = -gamepad2.right_stick_y;
//
//        if (Math.abs(stick) > 0.1) {
//            mechanisms.moveHoodManual(stick * 0.8);
//        } else {
//            mechanisms.stopHood(); // This will engage PID holding automatically
//        }
//
//        // Preset hood movements with D-pad - use fixed power values
//        boolean dpadUp = gamepad2.dpad_up;
//        boolean dpadDown = gamepad2.dpad_down;
//
//        if (dpadUp && !previousG2_DpadUp) {
//            mechanisms.moveHoodManual(0.6);
//        }
//        if (dpadDown && !previousG2_DpadDown) {
//            mechanisms.moveHoodManual(-0.6);
//        }
//
//        // Stop hood when D-pad buttons are released (engages PID holding)
//        if (previousG2_DpadUp && !dpadUp) {
//            mechanisms.stopHood();
//        }
//        if (previousG2_DpadDown && !dpadDown) {
//            mechanisms.stopHood();
//        }
//
//        previousG2_DpadUp = dpadUp;
//        previousG2_DpadDown = dpadDown;
    }

    private void handleTurretControls() {
        // Turret control with left stick X - direct power control
        double stick = gamepad2.left_stick_x;

        if (Math.abs(stick) > 0.1) {
            mechanisms.moveTurretManual(stick * 0.8);
        } else {
            mechanisms.stopTurret();
        }

        // Preset turret movements with D-pad - use fixed power values
        boolean dpadLeft = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;

        if (dpadLeft && !previousG2_DpadLeft) {
            mechanisms.moveTurretManual(-0.6);
        }
        if (dpadRight && !previousG2_DpadRight) {
            mechanisms.moveTurretManual(0.6);
        }

        // Stop turret when D-pad buttons are released
        if (previousG2_DpadLeft && !dpadLeft) {
            mechanisms.stopTurret();
        }
        if (previousG2_DpadRight && !dpadRight) {
            mechanisms.stopTurret();
        }

        // Stop turret with Back button
        boolean backPressed = gamepad2.back;
        if (backPressed && !previousG2_Back) {
            mechanisms.stopTurret();
        }
        previousG2_Back = backPressed;

        previousG2_DpadLeft = dpadLeft;
        previousG2_DpadRight = dpadRight;
    }

    private void handleContinuousServoControls() {
        // Control continuous servo 1 with gamepad1 triggers
        if (gamepad1.right_trigger > 0.1) {
            mechanisms.setContinuousServo1Power(0.5);
        } else if (gamepad1.left_trigger > 0.1) {
            mechanisms.setContinuousServo1Power(-0.5);
        } else {
            mechanisms.setContinuousServo1Power(0);
        }

        // Control continuous servo 2 with gamepad2 triggers
        if (gamepad2.right_trigger > 0.1) {
            mechanisms.setContinuousServo2Power(0.5);
        } else if (gamepad2.left_trigger > 0.1) {
            mechanisms.setContinuousServo2Power(-0.5);
        } else {
            mechanisms.setContinuousServo2Power(0);
        }

        // Use left stick buttons for precise servo positioning
        boolean leftStick = gamepad1.left_stick_button;
        boolean rightStick = gamepad1.right_stick_button;

        if (leftStick && !previousG1_LeftStick) {
            mechanisms.moveContinuousServo1Revolutions(0.25);
        }
        if (rightStick && !previousG1_RightStick) {
            mechanisms.moveContinuousServo1Revolutions(-0.25);
        }

        previousG1_LeftStick = leftStick;
        previousG1_RightStick = rightStick;
    }

    private void handleUtilityControls() {
        // Toggle limit switch functionality with D-pad Up
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !previousG1_DpadUp) {
            mechanisms.setUseLimitSwitch(!mechanisms.isUseLimitSwitch());
            telemetry.addLine("Limit switch: " +
                    (mechanisms.isUseLimitSwitch() ? "ENABLED" : "DISABLED"));
        }
        previousG1_DpadUp = dpadUp;

        // Manual home with D-pad Down
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !previousG1_DpadDown && mechanisms.isUseLimitSwitch()) {
            mechanisms.homeSpindexer();
        }
        previousG1_DpadDown = dpadDown;

        // Reset all encoders with Start + Back buttons
        boolean startPressed = gamepad1.start;
        boolean backPressed = gamepad1.back;

        if (startPressed && backPressed && (!previousG1_Start || !previousG1_Back)) {
            mechanisms.resetAllEncoders();
            telemetry.addLine("All encoders reset!");
        }
        previousG1_Start = startPressed;
        previousG1_Back = backPressed;
    }

    private void handleServoTestControls() {
        // Test ALL servos with Gamepad1 X button
        boolean xPressed = gamepad1.x;
        if (xPressed && !previousG1_X) {
            mechanisms.testAllServos();
            telemetry.addLine("ALL SERVOS TESTED - Check if they moved!");
        }
        previousG1_X = xPressed;

        // Individual servo testing with Gamepad1 A button
        boolean aPressed = gamepad1.a;
        if (aPressed && !previousG1_A) {
            mechanisms.setHoodTestPosition(0.8);
            mechanisms.setTurretTestPosition(0.8);
            telemetry.addLine("Servos set to 0.8 position");
        }
        previousG1_A = aPressed;

        // Quick manual controls with D-pad
        if (gamepad1.dpad_up) {
            mechanisms.setHoodTestPosition(0.8);
            telemetry.addLine("Hood: 0.8");
        }
        if (gamepad1.dpad_down) {
            mechanisms.setHoodTestPosition(0.2);
            telemetry.addLine("Hood: 0.2");
        }
        if (gamepad1.dpad_right) {
            mechanisms.setTurretTestPosition(0.8);
            telemetry.addLine("Turret: 0.8");
        }
        if (gamepad1.dpad_left) {
            mechanisms.setTurretTestPosition(0.2);
            telemetry.addLine("Turret: 0.2");
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1fs", runtime.seconds());
        telemetry.addData("Status", "All systems operational");
        mechanisms.addTelemetryData(telemetry);
        mechanisms.debugHoodSystem(telemetry);
        mechanisms.debugServoSystem(telemetry);

        aprilTagDetector.addTelemetry(telemetry);

        // Add control hints
        telemetry.addLine("Distance: "+ aprilTagDetector.getDistance());
        telemetry.addLine("");
        telemetry.addLine("=== Controls ===");
        telemetry.addData("Drive", "LS: Move, RS: Rotate, Y: Toggle Field-Centric, B: Reset Heading");
        telemetry.addData("Speed", "%.1fx (G1 LB/RB)", driveSpeedModifier);
        telemetry.addData("Intake", "RT: Forward, LT: Reverse");
        telemetry.addData("Spindexer", "AUTO: Ball detection & rotation | A: Manual Advance | X: Manual Step | Y: Home");
        telemetry.addData("Launcher", "RB: +RPM, LB: -RPM, Start: Stop");
        telemetry.addData("Hood", "RS: Manual Power (PID holds when released) | Dpad Up/Down: Move");
        telemetry.addData("Turret", "LS: Manual Power, Dpad Left/Right: Move, Back: Stop");
        telemetry.addData("Servos", "Triggers: Manual, Stick Buttons: Precise");
        telemetry.addData("Reset", "Start+Back: Reset Encoders");
        telemetry.addData("Servo Test", "G1 X: Test All, G1 A: Set 0.8, Dpad: Manual");

        telemetry.update();
    }
}