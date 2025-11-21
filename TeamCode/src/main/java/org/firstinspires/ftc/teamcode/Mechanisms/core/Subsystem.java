package org.firstinspires.ftc.teamcode.Mechanisms.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Defines the standard contract for a robot subsystem.
 * Each subsystem represents a major mechanical component of the robot
 * and must be able to be updated, stopped, and provide telemetry data.
 */
public interface Subsystem {

    /**
     * Initializes the subsystem. This method should be called from the main DECODEMechanisms constructor.
     * This is where hardware devices are retrieved from the hardware map.
     */
    // void init(HardwareMap hardwareMap);

    /**
     * Updates the state of the subsystem.
     * This method is called repeatedly in the main robot loop and should contain
     * all logic for state transitions, motor power calculations, and sensor processing.
     */
    void update();

    /**
     * Brings the subsystem to a complete and safe stop.
     * This method is called at the end of an OpMode to ensure all motors are off
     * and all background threads are properly terminated.
     */
    void stop();

    /**
     * Appends telemetry data for this subsystem to the provided Telemetry object.
     *
     * @param telemetry The Telemetry object to which data should be added.
     */
    void addTelemetryData(Telemetry telemetry);
}
