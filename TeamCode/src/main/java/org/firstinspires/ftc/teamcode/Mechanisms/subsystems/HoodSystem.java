package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.PIDController;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * The HoodSystem manages the adjustable hood for changing the launcher's angle.
 * It uses a PID controller to hold a specific position and allows for manual control.
 */
public class HoodSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final double MIN_POS = 0.1;
    private static final double MAX_POS = 1.0;
    private static final double POSITION_TOLERANCE = 0.02;
    private static final double HOLD_POWER_LIMIT = 0.3;
    private static final double MANUAL_POWER_MULTIPLIER = 0.8;
    private static final double MANUAL_POWER_DEADZONE = 0.1;
    private static final long THREAD_SLEEP_MS = 50;
    private static final long SHUTDOWN_TIMEOUT_MS = 100;

    // PID Coefficients
    private static final double KP = 0.8, KI = 0.05, KD = 0.1;

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final CRServo hoodServo;
    private final AnalogInput hoodEncoder;

    // ==================================================
    // S T A T E
    // ==================================================
    private final PIDController pidController;
    private final ExecutorService encoderExecutorService;
    private volatile double currentPosition = 0.5;
    private double targetPosition = 0.5;

    private enum State { MANUAL, HOLDING, MOVING_TO_POSITION }
    private State currentState = State.HOLDING;

    /**
     * Constructor for HoodSystem.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public HoodSystem(HardwareMap hardwareMap) {
        try {
            hoodServo = hardwareMap.get(CRServo.class, "hood");
            hoodEncoder = hardwareMap.get(AnalogInput.class, "hoodEnc");
            pidController = new PIDController(KP, KI, KD);

            encoderExecutorService = Executors.newSingleThreadExecutor();
            encoderExecutorService.submit(this::readEncoderContinuously);

            // Initialize target position to the current hardware state.
            targetPosition = getPosition();

            // OPTIONAL FIX: Initialize servo to stopped state
            hoodServo.setPower(0);
        } catch (Exception e) {
            throw new RuntimeException("Hood system initialization failed", e);
        }
    }

    /**
     * Continuously reads the hood encoder value on a background thread.
     */
    private void readEncoderContinuously() {
        while (!Thread.currentThread().isInterrupted()) {
            currentPosition = hoodEncoder != null ? Math.max(0.0, Math.min(1.0, hoodEncoder.getVoltage() / 3.3)) : 0.5;
            try {
                Thread.sleep(THREAD_SLEEP_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    /**
     * Main update loop for the hood.
     */
    @Override
    public void update() {
        switch (currentState) {
            case MANUAL:
                // Manual power is set directly via setPower(), no periodic logic needed.
                break;
            case HOLDING:
            case MOVING_TO_POSITION:
                double pidPower = pidController.calculate(currentPosition, targetPosition);
                pidPower = Math.max(-HOLD_POWER_LIMIT, Math.min(HOLD_POWER_LIMIT, pidPower));

                if (Math.abs(currentPosition - targetPosition) > POSITION_TOLERANCE) {
                    hoodServo.setPower(pidPower);
                } else {
                    hoodServo.setPower(0);
                    if (currentState == State.MOVING_TO_POSITION) {
                        setState(State.HOLDING); // Position reached, switch to holding.
                    }
                }
                break;
        }
    }

    @Override
    public void stop() {
        setPower(0); // This will transition the state to HOLDING and stop the motor.
        if (encoderExecutorService != null) {
            encoderExecutorService.shutdownNow();
            try {
                if (!encoderExecutorService.awaitTermination(SHUTDOWN_TIMEOUT_MS, TimeUnit.MILLISECONDS)) {
                    System.err.println("Hood encoder thread did not terminate gracefully.");
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Hood Position", "%.3f", currentPosition);
        telemetry.addData("Hood Target", "%.3f", targetPosition);
        telemetry.addData("Hood State", currentState);
    }

    /**
     * Sets the raw power of the hood servo for manual control.
     * If power is near zero, the system will transition to holding the last known position.
     *
     * @param power The power to apply (-1.0 to 1.0).
     */
    public void setPower(double power) {
        if (Math.abs(power) > MANUAL_POWER_DEADZONE) {
            setState(State.MANUAL);
            hoodServo.setPower(power * MANUAL_POWER_MULTIPLIER);
        } else {
            if (currentState == State.MANUAL) {
                holdCurrentPosition(); // Transition from manual to holding.
            }
        }
    }

    /**
     * Commands the hood to hold its current position using PID control.
     */
    public void holdCurrentPosition() {
        setPosition(currentPosition);
    }

    /**
     * Commands the hood to move to and hold a specific position.
     *
     * @param position The target position (0.0 to 1.0).
     */
    public void setPosition(double position) {
        this.targetPosition = Math.max(MIN_POS, Math.min(MAX_POS, position));
        setState(State.MOVING_TO_POSITION);
    }

    /**
     * Changes the internal state of the hood.
     */
    private void setState(State newState) {
        if (currentState != newState) {
            this.currentState = newState;
            this.pidController.reset();
        }
    }

    // ==================================================
    // G E T T E R S
    // ==================================================

    public double getPosition() {
        return currentPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}