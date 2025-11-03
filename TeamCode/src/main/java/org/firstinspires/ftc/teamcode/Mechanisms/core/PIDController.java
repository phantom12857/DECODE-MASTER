package org.firstinspires.ftc.teamcode.Mechanisms.core;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Generic PID controller implementation
 */
public class PIDController {
    private double kp;
    private double ki;
    private double kd;
    private double integral = 0;
    private double previousError = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private double maxIntegral = 1000;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        timer.reset();
    }

    public double calculate(double current, double target) {
        double error = target - current;
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) dt = 0.01;

        integral += error * dt;
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

        double derivative = (error - previousError) / dt;
        previousError = error;

        return (kp * error) + (ki * integral) + (kd * derivative);
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        timer.reset();
    }

    // Getters and setters
    public void setCoefficients(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        reset();
    }

    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = maxIntegral;
    }
}