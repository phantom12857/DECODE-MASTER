package org.firstinspires.ftc.teamcode.Mechanisms.utils;

/**
 * Simple low-pass filter for sensor data
 */
public class Filter {
    private final double alpha;
    private double filteredValue;
    private boolean initialized = false;

    public Filter(double alpha) {
        this.alpha = alpha;
    }

    public double update(double newValue) {
        if (!initialized) {
            filteredValue = newValue;
            initialized = true;
        } else {
            filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
        }
        return filteredValue;
    }

    public void reset() {
        initialized = false;
    }

    public double getValue() {
        return filteredValue;
    }
}