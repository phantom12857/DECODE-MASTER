package org.firstinspires.ftc.teamcode.Mechanisms.utils;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * The LimitSwitchManager class provides a debounced, thread-safe interface to a digital limit switch.
 */
public class LimitSwitchManager {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final long THREAD_SLEEP_MS = 20;
    private static final long DEBOUNCE_DELAY_MS = 50;
    private static final long SHUTDOWN_TIMEOUT_MS = 100;

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final DigitalChannel limitSwitch;

    // ==================================================
    // S T A T E
    // ==================================================
    private final ExecutorService executorService;
    private volatile boolean isPressed = false;

    /**
     * Constructor for LimitSwitchManager.
     *
     * @param hardwareMap The robot's hardware map.
     * @param deviceName  The name of the limit switch in the hardware configuration.
     */
    public LimitSwitchManager(HardwareMap hardwareMap, String deviceName) {
        try {
            limitSwitch = hardwareMap.get(DigitalChannel.class, deviceName);
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
            executorService = Executors.newSingleThreadExecutor();
            executorService.submit(this::readSwitchContinuously);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize LimitSwitchManager: " + deviceName, e);
        }
    }

    /**
     * Continuously reads the limit switch state on a background thread, with debouncing.
     */
    private void readSwitchContinuously() {
        long lastDebounceTime = 0;
        boolean lastReading = false;

        while (!Thread.currentThread().isInterrupted()) {
            boolean currentReading = !limitSwitch.getState(); // Inverted: getState() is true when open.

            if (currentReading != lastReading) {
                lastDebounceTime = System.currentTimeMillis();
            }

            if ((System.currentTimeMillis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
                isPressed = currentReading;
            }

            lastReading = currentReading;

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
                    System.err.println("Limit switch thread did not terminate gracefully.");
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    // ==================================================
    // G E T T E R S
    // ==================================================

    public boolean isPressed() {
        return isPressed;
    }
}
