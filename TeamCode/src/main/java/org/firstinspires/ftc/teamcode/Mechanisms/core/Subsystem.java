package org.firstinspires.ftc.teamcode.Mechanisms.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Base interface for all subsystems
 */
public interface Subsystem {
    void update();
    void stop();
    void addTelemetryData(Telemetry telemetry);
}