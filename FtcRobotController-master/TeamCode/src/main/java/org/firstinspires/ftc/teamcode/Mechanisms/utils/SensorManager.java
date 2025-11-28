package org.firstinspires.ftc.teamcode.Mechanisms.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * The SensorManager class provides a filtered, thread-safe interface to a color/distance sensor.
 * It is now capable of identifying the color of detected game pieces.
 */
public class SensorManager {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final int GREEN_THRESHOLD_G = 120;
    private static final int PURPLE_THRESHOLD_R = 100;
    private static final int PURPLE_THRESHOLD_B = 100;
    private static final long THREAD_SLEEP_MS = 50;
    private static final long SHUTDOWN_TIMEOUT_MS = 100;
    private static final double FILTER_ALPHA = 0.3;

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;

    // ==================================================
    // S T A T E
    // ==================================================
    private final ExecutorService executorService;
    private volatile double filteredDistance = Double.MAX_VALUE;
    private volatile GamePiece.Color detectedColor = GamePiece.Color.NONE;
    private final double ballDetectionDistanceMM;

    /**
     * Constructor for SensorManager.
     */
    public SensorManager(HardwareMap hardwareMap, String deviceName, double ballDetectionDistanceMM) {
        this.ballDetectionDistanceMM = ballDetectionDistanceMM;
        try {
            colorSensor = hardwareMap.get(ColorSensor.class, deviceName);
            distanceSensor = hardwareMap.get(DistanceSensor.class, deviceName);
            executorService = Executors.newSingleThreadExecutor();
            executorService.submit(this::readSensorsContinuously);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize SensorManager: " + deviceName, e);
        }
    }

    /**
     * Continuously reads and filters sensor data on a background thread.
     */
    private void readSensorsContinuously() {
        Filter distanceFilter = new Filter(FILTER_ALPHA);
        Filter redFilter = new Filter(FILTER_ALPHA);
        Filter greenFilter = new Filter(FILTER_ALPHA);
        Filter blueFilter = new Filter(FILTER_ALPHA);

        while (!Thread.currentThread().isInterrupted()) {
            filteredDistance = distanceFilter.update(distanceSensor.getDistance(DistanceUnit.MM));
            int red = (int) redFilter.update(colorSensor.red());
            int green = (int) greenFilter.update(colorSensor.green());
            int blue = (int) blueFilter.update(colorSensor.blue());

            if (filteredDistance < ballDetectionDistanceMM) {
                if (green > GREEN_THRESHOLD_G && green > red && green > blue) {
                    detectedColor = GamePiece.Color.GREEN;
                } else if (red > PURPLE_THRESHOLD_R && blue > PURPLE_THRESHOLD_B && (red + blue) > (green * 2)) {
                    detectedColor = GamePiece.Color.PURPLE;
                } else {
                    detectedColor = GamePiece.Color.NONE; // Detected, but not a valid color
                }
            } else {
                detectedColor = GamePiece.Color.NONE;
            }

            try {
                Thread.sleep(THREAD_SLEEP_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    /**
     * Stops the background thread.
     */
    public void stop() {
        if (executorService != null) {
            executorService.shutdownNow();
            try {
                if (!executorService.awaitTermination(SHUTDOWN_TIMEOUT_MS, TimeUnit.MILLISECONDS)) {
                    System.err.println("Sensor manager thread did not terminate gracefully.");
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    // ==================================================
    // G E T T E R S
    // ==================================================

    public GamePiece.Color getDetectedColor() {
        return detectedColor;
    }

    public boolean isBallDetected() {
        return detectedColor != GamePiece.Color.NONE;
    }
}
