package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.core.DECODEMechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.AprilTagDetector;

@TeleOp(name = "DECODETeleOp", group = "Linear OpMode")
public class DECODETeleOp1ManArmy extends LinearOpMode {
    private DECODEMechanisms mechanisms;
    private AprilTagDetector aprilTagDetector;
    private final ElapsedTime runtime = new ElapsedTime();
    private TeleOpController1ManArmy controller;

    @Override
    public void runOpMode() {
        // Initialize mechanisms
        mechanisms = new DECODEMechanisms(hardwareMap);
        aprilTagDetector = new AprilTagDetector(hardwareMap);
        controller = new TeleOpController1ManArmy(mechanisms, aprilTagDetector, gamepad1, gamepad2, telemetry);

        telemetry.addLine("Initialized, waiting for start...");
        telemetry.addLine("Spindexer limit switch: " +
                (mechanisms.spindexer.isLimitPressed() ? "PRESSED" : "Released"));
        telemetry.addLine("Auto-Intake: ENABLED by default");
        telemetry.addLine("Hood PID Holding: ACTIVE");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Auto-home spindexer on start
        mechanisms.spindexer.home();

        // === Main loop ===
        while (opModeIsActive()) {
            controller.update();
            mechanisms.update();
            updateTelemetry();
        }

        mechanisms.stopAll();
    }

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1fs", runtime.seconds());
        telemetry.addData("Status", "All systems operational");
        mechanisms.addTelemetryData(telemetry);
        controller.addTelemetryData(telemetry);
        telemetry.update();
    }
}