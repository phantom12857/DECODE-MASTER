package org.firstinspires.ftc.teamcode.Mechanisms.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.subsystems.*;

import java.util.ArrayList;
import java.util.List;

/**
 * The DECODEMechanisms class is the central orchestrator for all robot subsystems.
 * It is responsible for initializing, updating, and stopping all mechanical components
 * in a structured and predictable manner.
 */
public class DECODEMechanisms {

    // ==================================================
    // M E C H A N I S M   R E G I S T R Y
    // ==================================================
    public final DriveSystem drive;
    public final SpindexerSystem spindexer;
    public final LauncherSystem launcher;
    public final IntakeSystem intake;
    public final HoodSystem hood;
    public final TurretSystem turret;

    private final List<Subsystem> allSubsystems;

    /**
     * Constructor for DECODEMechanisms.
     *
     * @param hardwareMap The robot's hardware map, used to initialize subsystems.
     */
    public DECODEMechanisms(HardwareMap hardwareMap) {
        allSubsystems = new ArrayList<>();

        // The order of initialization should be considered if some subsystems
        // depend on others, though this is not currently the case.
        drive = new DriveSystem(hardwareMap);
        spindexer = new SpindexerSystem(hardwareMap);
        launcher = new LauncherSystem(hardwareMap);
        intake = new IntakeSystem(hardwareMap);
        hood = new HoodSystem(hardwareMap);
        turret = new TurretSystem(hardwareMap);

        // Register all subsystems for centralized management.
        allSubsystems.add(drive);
        allSubsystems.add(spindexer);
        allSubsystems.add(launcher);
        allSubsystems.add(intake);
        allSubsystems.add(hood);
        allSubsystems.add(turret);
    }

    /**
     * Calls the update() method on all registered subsystems.
     * This is the main entry point for the periodic logic of the robot's mechanisms.
     */
    public void update() {
        for (Subsystem subsystem : allSubsystems) {
            try {
                subsystem.update();
            } catch (Exception e) {
                // Log or handle exceptions from a specific subsystem to prevent a full crash.
                System.err.println("Exception in subsystem update: " + subsystem.getClass().getSimpleName());
                e.printStackTrace();
            }
        }
    }

    /**
     * Calls the stop() method on all registered subsystems.
     * This ensures a graceful shutdown of all mechanical and background processes.
     */
    public void stopAll() {
        for (Subsystem subsystem : allSubsystems) {
            try {
                subsystem.stop();
            } catch (Exception e) {
                System.err.println("Exception in subsystem stop: " + subsystem.getClass().getSimpleName());
                e.printStackTrace();
            }
        }
    }

    /**
     * Aggregates and displays telemetry data from all registered subsystems.
     *
     * @param telemetry The Telemetry object to which all data will be added.
     */
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addLine("--- Subsystem Telemetry ---");
        for (Subsystem subsystem : allSubsystems) {
            subsystem.addTelemetryData(telemetry);
            telemetry.addLine(); // Separator for readability
        }
    }
}
