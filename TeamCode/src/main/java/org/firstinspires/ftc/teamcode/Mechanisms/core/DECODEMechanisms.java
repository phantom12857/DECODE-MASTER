package org.firstinspires.ftc.teamcode.Mechanisms.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.subsystems.*;

/**
 * Main robot mechanisms orchestrator that manages all subsystems
 */
public class DECODEMechanisms {
    // Subsystem instances
    public final DriveSystem drive;
    public final SpindexerSystem spindexer;
    public final LauncherSystem launcher;
    public final IntakeSystem intake;
    public final HoodSystem hood;
    public final TurretSystem turret;
    public final ContinuousServoSystem continuousServos;

    public DECODEMechanisms(HardwareMap hardwareMap) {
        // Initialize subsystems
        this.drive = new DriveSystem(hardwareMap);
        this.spindexer = new SpindexerSystem(hardwareMap);
        this.launcher = new LauncherSystem(hardwareMap);
        this.intake = new IntakeSystem(hardwareMap);
        this.hood = new HoodSystem(hardwareMap);
        this.turret = new TurretSystem(hardwareMap);
        this.continuousServos = new ContinuousServoSystem(hardwareMap);
    }

    /**
     * Update all subsystems
     */
    public void update() {
        spindexer.update();
        launcher.update();
        continuousServos.update();
        // Hood PID is handled automatically in setHoodPower
    }

    /**
     * Stop all subsystems
     */
    public void stopAll() {
        drive.stop();
        intake.stop();
        spindexer.stop();
        launcher.stop();
        hood.stop();
        turret.stop();
        continuousServos.stop();
    }

    /**
     * Add telemetry data from all subsystems
     */
    public void addTelemetryData(Telemetry telemetry) {
        drive.addTelemetryData(telemetry);
        spindexer.addTelemetryData(telemetry);
        launcher.addTelemetryData(telemetry);
        intake.addTelemetryData(telemetry);
        hood.addTelemetryData(telemetry);
        turret.addTelemetryData(telemetry);
        continuousServos.addTelemetryData(telemetry);
    }
}