// UPDATED: DECODEMechanisms.java
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
 * <p>
 * REFACTORED: Added ContinuousServoSystem registration and enhanced error handling
 */
public class DECODEMechanisms {

    public final HardwareMap hardwareMap;

    // ==================================================
    // M E C H A N I S M   R E G I S T R Y
    // ==================================================
    public final DriveSystem drive;
    public final SpindexerSystem spindexer;
    public final LauncherSystem launcher;
    public final IntakeSystem intake;
    public final HoodSystem hood;
    public final TurretSystem turret;
    public final ContinuousServoSystem continuousServos;  // FIXED: Now registered!

    private final List<Subsystem> allSubsystems;
    private final Telemetry telemetry;
    private boolean isInitialized = false;
    private final List<String> initializationErrors = new ArrayList<>();

    /**
     * Constructor for DECODEMechanisms.
     *
     * @param hardwareMap The robot's hardware map, used to initialize subsystems.
     */
    public DECODEMechanisms(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    /**
     * Constructor for DECODEMechanisms with telemetry.
     *
     * @param hardwareMap The robot's hardware map, used to initialize subsystems.
     * @param telemetry The telemetry object for status messages.
     */
    public DECODEMechanisms(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        allSubsystems = new ArrayList<>();

        // Initialize all subsystems with error handling
        // Order matters if subsystems depend on each other
        drive = initializeSubsystem("Drive", () -> new DriveSystem(hardwareMap));
        spindexer = initializeSubsystem("Spindexer", () -> new SpindexerSystem(hardwareMap));
        launcher = initializeSubsystem("Launcher", () -> new LauncherSystem(hardwareMap));
        intake = initializeSubsystem("Intake", () -> new IntakeSystem(hardwareMap));
        hood = initializeSubsystem("Hood", () -> new HoodSystem(hardwareMap));
        turret = initializeSubsystem("Turret", () -> new TurretSystem(hardwareMap));
        continuousServos = initializeSubsystem("ContinuousServos", () -> new ContinuousServoSystem(hardwareMap));  // FIXED: Now initialized!

        // Register all successfully initialized subsystems for centralized management
        if (drive != null) allSubsystems.add(drive);
        if (spindexer != null) allSubsystems.add(spindexer);
        if (launcher != null) allSubsystems.add(launcher);
        if (intake != null) allSubsystems.add(intake);
        if (hood != null) allSubsystems.add(hood);
        if (turret != null) allSubsystems.add(turret);
        if (continuousServos != null) allSubsystems.add(continuousServos);  // FIXED: Now registered!

        isInitialized = initializationErrors.isEmpty();

        if (!isInitialized && telemetry != null) {
            telemetry.addLine("⚠️ INITIALIZATION WARNINGS:");
            for (String error : initializationErrors) {
                telemetry.addLine("  " + error);
            }
            telemetry.update();
        }
    }

    /**
     * Helper method to initialize subsystems with error handling.
     *
     * @param name The name of the subsystem for error reporting
     * @param initializer Lambda that creates the subsystem
     * @return The initialized subsystem, or null if initialization failed
     */
    private <T extends Subsystem> T initializeSubsystem(String name, SubsystemInitializer<T> initializer) {
        try {
            T subsystem = initializer.initialize();
            if (telemetry != null) {
                telemetry.addLine("✓ " + name + " initialized");
                telemetry.update();
            }
            return subsystem;
        } catch (Exception e) {
            String errorMsg = name + " initialization failed: " + e.getMessage();
            initializationErrors.add(errorMsg);
            
            if (telemetry != null) {
                telemetry.addData("⚠️ Init Failed", name);
                telemetry.addData("Error", e.getClass().getSimpleName());
            }
            
            return null;
        }
    }

    /**
     * Functional interface for subsystem initialization
     */
    @FunctionalInterface
    private interface SubsystemInitializer<T> {
        T initialize() throws Exception;
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
                // Log error without crashing to prevent complete robot shutdown
                String subsystemName = subsystem.getClass().getSimpleName();
                if (telemetry != null) {
                    telemetry.addData("⚠️ Subsystem Error", subsystemName);
                }
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
                // Errors during shutdown are logged but don't prevent other subsystems from stopping
                if (telemetry != null) {
                    telemetry.addData("⚠️ Stop Error", subsystem.getClass().getSimpleName());
                }
            }
        }
    }

    /**
     * Emergency stop - immediately halts all mechanisms.
     * Call this in error conditions or when emergency stop is pressed.
     */
    public void emergencyStop() {
        System.err.println("🚨 EMERGENCY STOP TRIGGERED 🚨");

        // Stop drive immediately
        if (drive != null) {
            try {
                drive.stop();
            } catch (Exception e) {
                System.err.println("Failed to emergency stop drive");
            }
        }

        // Stop all other subsystems
        stopAll();

        if (telemetry != null) {
            telemetry.addLine("🚨 EMERGENCY STOP 🚨");
            telemetry.update();
        }
    }

    /**
     * Aggregates and displays telemetry data from all registered subsystems.
     *
     * @param telemetry The Telemetry object to which all data will be added.
     */
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addLine("=== DECODE Subsystems ===");
        telemetry.addData("Initialized", isInitialized ? "✓ YES" : "⚠️ PARTIAL");
        telemetry.addData("Active Subsystems", allSubsystems.size() + "/7");
        telemetry.addLine();

        for (Subsystem subsystem : allSubsystems) {
            try {
                subsystem.addTelemetryData(telemetry);
                telemetry.addLine(); // Separator for readability
            } catch (Exception e) {
                telemetry.addData("⚠️ " + subsystem.getClass().getSimpleName(), "Telemetry Error");
            }
        }

        if (!initializationErrors.isEmpty()) {
            telemetry.addLine("--- Initialization Warnings ---");
            for (String error : initializationErrors) {
                telemetry.addLine(error);
            }
        }
    }

    /**
     * Checks if all subsystems initialized successfully.
     *
     * @return true if all subsystems are initialized, false otherwise
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Gets the list of initialization errors (if any).
     *
     * @return List of error messages from failed subsystem initializations
     */
    public List<String> getInitializationErrors() {
        return new ArrayList<>(initializationErrors);
    }

    /**
     * Checks if a specific subsystem is available.
     *
     * @return true if the subsystem is initialized and available
     */
    public boolean hasDrive() { return drive != null; }
    public boolean hasSpindexer() { return spindexer != null; }
    public boolean hasLauncher() { return launcher != null; }
    public boolean hasIntake() { return intake != null; }
    public boolean hasHood() { return hood != null; }
    public boolean hasTurret() { return turret != null; }
    public boolean hasContinuousServos() { return continuousServos != null; }  // NEW: Availability check
}