package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;

/**
 * This is the main TeleOp OpMode for the robot.
 * It orchestrates the initialization and main loop, delegating control logic
 * to the TeleOpController class.
 */
@TeleOp(name = "DECODE TeleOp", group = "Linear OpMode")
public class DECODETeleOp extends LinearOpMode {

    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;
    private TeleOpController controller;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        try {
            // Initialize all robot hardware and controllers.
            mechanisms = new DECODEMechanisms(hardwareMap);
            aprilTagDetector = new AprilTagDetector(hardwareMap);
            controller = new TeleOpController(mechanisms, aprilTagDetector, gamepad1, gamepad2, telemetry);

            telemetry.addLine("Robot Initialized. Waiting for start...");
            telemetry.update();

            waitForStart();
            runtime.reset();

            // Main robot loop.
            while (opModeIsActive()) {
                controller.update();
                mechanisms.update();
                aprilTagDetector.update();
                updateTelemetry();
            }
        } finally {
            // Ensure graceful shutdown of all mechanisms and resources.
            if (mechanisms != null) {
                mechanisms.stopAll();
            }
            if (aprilTagDetector != null) {
                aprilTagDetector.close();
            }
        }
    }

    /**
     * Updates and displays telemetry data during the OpMode.
     */
    private void updateTelemetry() {
        telemetry.addData("Runtime", runtime.toString());
        telemetry.addLine();
        mechanisms.addTelemetryData(telemetry);
        telemetry.addLine();
        controller.addTelemetryData(telemetry);
        telemetry.addLine();
        aprilTagDetector.addTelemetry(telemetry);
        telemetry.update();
    }
}
