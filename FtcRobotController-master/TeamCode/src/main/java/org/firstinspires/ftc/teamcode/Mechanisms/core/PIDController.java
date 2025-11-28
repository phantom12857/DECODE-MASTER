package org.firstinspires.ftc.teamcode.Mechanisms.core;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Implements a standard Proportional-Integral-Derivative (PID) controller.
 * This class is used to calculate the necessary output to bring a system's state
 * to a desired setpoint by minimizing the error over time.
 */
public class PIDController {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final double DEFAULT_MAX_INTEGRAL = 1000.0;
    private static final double MIN_TIME_DELTA_SECONDS = 0.001;

    // ==================================================
    // P I D   C O E F F I C I E N T S
    // ==================================================
    private double kP;
    private double kI;
    private double kD;

    // ==================================================
    // C O N T R O L L E R   S T A T E
    // ==================================================
    private double integral = 0.0;
    private double previousError = 0.0;
    private double maxIntegral = DEFAULT_MAX_INTEGRAL;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor for the PIDController.
     *
     * @param kP The Proportional gain.
     * @param kI The Integral gain.
     * @param kD The Derivative gain.
     */
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.timer.reset();
    }

    /**
     * Calculates the control output based on the current and target values.
     *
     * @param current The current measured value of the system.
     * @param target  The desired setpoint for the system.
     * @return The calculated control output.
     */
    public double calculate(double current, double target) {
        double error = target - current;
        double dt = timer.seconds();
        timer.reset();

        // Prevent division by zero or extreme values if the loop is too fast.
        if (dt < MIN_TIME_DELTA_SECONDS) {
            dt = MIN_TIME_DELTA_SECONDS;
        }

        // Integral term with anti-windup.
        integral += error * dt;
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

        // Derivative term.
        double derivative = (error - previousError) / dt;
        previousError = error;

        // Proportional term.
        double proportional = error;

        return (kP * proportional) + (kI * integral) + (kD * derivative);
    }

    /**
     * Resets the controller's internal state (integral, previous error, and timer).
     * This should be called when the controller is disabled or the target is changed significantly.
     */
    public void reset() {
        integral = 0.0;
        previousError = 0.0;
        timer.reset();
    }

    /**
     * Sets new PID coefficients for the controller.
     *
     * @param kP The new Proportional gain.
     * @param kI The new Integral gain.
     * @param kD The new Derivative gain.
     */
    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        reset();
    }

    /**
     * Sets the maximum value for the integral term to prevent integral windup.
     *
     * @param maxIntegral The maximum absolute value for the integral term.
     */
    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = Math.abs(maxIntegral);
    }
}
