package org.firstinspires.ftc.teamcode.Mechanisms.utils;

/**
 * A simple low-pass filter for smoothing sensor data.
 * This filter reduces the effect of rapid fluctuations or noise in sensor readings.
 */
public class Filter {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private final double alpha;

    // ==================================================
    // S T A T E
    // ==================================================
    private double filteredValue;
    private boolean isInitialized = false;

    /**
     * Constructor for the Filter.
     *
     * @param alpha The smoothing factor, a value between 0.0 and 1.0.
     *              A lower alpha results in more smoothing but slower response.
     *              A higher alpha results in less smoothing but faster response.
     */
    public Filter(double alpha) {
        this.alpha = alpha;
    }

    /**
     * Updates the filter with a new raw sensor value.
     *
     * @param newValue The latest raw value from the sensor.
     * @return The new filtered value.
     */
    public double update(double newValue) {
        if (!isInitialized) {
            filteredValue = newValue;
            isInitialized = true;
        } else {
            filteredValue = alpha * newValue + (1.0 - alpha) * filteredValue;
        }
        return filteredValue;
    }

    /**
     * Resets the filter, causing it to re-initialize on the next update.
     */
    public void reset() {
        isInitialized = false;
    }

    /**
     * Gets the current filtered value without updating it.
     *
     * @return The last calculated filtered value.
     */
    public double getValue() {
        return filteredValue;
    }
}
