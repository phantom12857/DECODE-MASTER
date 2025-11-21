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
 * It is primarily used for detecting game elements (e.g., balls).
 */
public class SensorManager {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final int COLOR_THRESHOLD = 100;
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
    private volatile int filteredRed = 0;
    private volatile int filteredGreen = 0;
    private volatile int filteredBlue = 0;
    private volatile boolean isBallDetected = false;
    private final double ballDetectionDistanceMM;

    /**
     * Constructor for SensorManager.
     *
     * @param hardwareMap             The robot's hardware map.
     * @param deviceName              The name of the sensor in the hardware configuration.
     * @param ballDetectionDistanceMM The distance in millimeters to consider a ball detected.
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
            filteredRed = (int) redFilter.update(colorSensor.red());
            filteredGreen = (int) greenFilter.update(colorSensor.green());
            filteredBlue = (int) blueFilter.update(colorSensor.blue());

            isBallDetected = filteredDistance < ballDetectionDistanceMM &&
                    (filteredRed > COLOR_THRESHOLD || filteredGreen > COLOR_THRESHOLD || filteredBlue > COLOR_THRESHOLD);

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

    public boolean isBallDetected() {
        return isBallDetected;
    }

    public double getBallDistance() {
        return filteredDistance;
    }

    public int getRed() {
        return filteredRed;
    }

    public int getGreen() {
        return filteredGreen;
    }

    public int getBlue() {
        return filteredBlue;
    }
}
