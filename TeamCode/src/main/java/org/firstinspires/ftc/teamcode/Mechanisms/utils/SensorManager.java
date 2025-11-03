package org.firstinspires.ftc.teamcode.Mechanisms.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Manages color and distance sensor data with filtering and ball detection logic
 */
public class SensorManager {
    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final Filter distanceFilter;
    private final Filter redFilter;
    private final Filter blueFilter;
    private final Filter greenFilter;

    private static final double BALL_DETECTION_DISTANCE_MM = 50;
    private static final int COLOR_THRESHOLD = 100;
    private static final int DEBOUNCE_COUNT = 3;

    private int ballDetectionCount = 0;
    private boolean lastBallDetected = false;

    public SensorManager(ColorSensor colorSensor, DistanceSensor distanceSensor) {
        this.colorSensor = colorSensor;
        this.distanceSensor = distanceSensor;
        this.distanceFilter = new Filter(0.3); // Low-pass filter for distance
        this.redFilter = new Filter(0.3);
        this.blueFilter = new Filter(0.3);
        this.greenFilter = new Filter(0.3);
    }

    public void update() {
        // Sensors are automatically updated when read
    }

    public boolean isBallDetected() {
        if (distanceSensor == null || colorSensor == null) {
            return false;
        }

        double filteredDistance = distanceFilter.update(distanceSensor.getDistance(DistanceUnit.MM));
        int filteredRed = (int) redFilter.update(colorSensor.red());
        int filteredBlue = (int) blueFilter.update(colorSensor.blue());
        int filteredGreen = (int) greenFilter.update(colorSensor.green());

        boolean ballDetected = filteredDistance < BALL_DETECTION_DISTANCE_MM &&
                (filteredRed > COLOR_THRESHOLD || filteredBlue > COLOR_THRESHOLD || filteredGreen > COLOR_THRESHOLD);

        // Debouncing
        if (ballDetected != lastBallDetected) {
            ballDetectionCount++;
            if (ballDetectionCount >= DEBOUNCE_COUNT) {
                lastBallDetected = ballDetected;
                ballDetectionCount = 0;
            }
        } else {
            ballDetectionCount = 0;
        }

        return lastBallDetected;
    }

    public double getBallDistance() {
        return distanceSensor != null ?
                distanceFilter.update(distanceSensor.getDistance(DistanceUnit.MM)) : 999;
    }

    public int getRed() {
        return colorSensor != null ? (int) redFilter.update(colorSensor.red()) : 0;
    }

    public int getBlue() {
        return colorSensor != null ? (int) blueFilter.update(colorSensor.blue()) : 0;
    }

    public int getGreen() {
        return colorSensor != null ? (int) greenFilter.update(colorSensor.green()) : 0;
    }

    public boolean isConnected() {
        return colorSensor != null && distanceSensor != null;
    }
}