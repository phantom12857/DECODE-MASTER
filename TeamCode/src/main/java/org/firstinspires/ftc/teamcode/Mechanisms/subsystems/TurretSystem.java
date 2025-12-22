// FIXED: TurretSystem.java with proper PID control for CRServo
package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.PIDController;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * The TurretSystem manages the rotating turret for aiming the launcher.
 * It provides manual control and PID-based position control for CRServo + encoder setup.
 *
 * FIXED: Added proper PID controller for CRServo position control
 */
public class TurretSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final double MANUAL_POWER_MULTIPLIER = 0.8;
    private static final long THREAD_SLEEP_MS = 50;
    private static final long SHUTDOWN_TIMEOUT_MS = 100;

    // Turret Limits and Encoder Configuration
    private static final double TURRET_MAX_ANGLE = 70.0;  // Degrees
    private static final double TICKS_PER_DEGREE = 10.0;  // Adjust based on your encoder and gear ratio

    // PID Tuning for CRServo position control
    private static final double KP_POSITION = 0.015;  // Proportional - increase for faster response
    private static final double KI_POSITION = 0.0001; // Integral - helps eliminate steady-state error
    private static final double KD_POSITION = 0.002;  // Derivative - dampens oscillation
    private static final double POSITION_TOLERANCE = 0.02; // Within 2% of full range is "at position"
    private static final double MAX_POSITION_POWER = 0.6;  // Limit maximum power for safety

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final CRServo turretServo;
    private final AnalogInput turretEncoder;
    private DcMotor turretMotor;  // If using motor with encoder instead of servo

    // ==================================================
    // S T A T E
    // ==================================================
    // Encoder reading happens on separate thread (READ-ONLY)
    // Hardware writes happen on main thread only (FTC Control Hub requirement)
    private final ExecutorService encoderExecutorService;

    // Volatile for thread-safe reads from background thread
    private volatile double currentPosition = 0.5;
    private volatile double currentAngleDegrees = 0.0;  // FIXED: Must be volatile - written by background thread, read by main thread

    // Position control state (main thread only)
    private enum ControlMode { MANUAL, POSITION_CONTROL }
    private ControlMode currentMode = ControlMode.MANUAL;
    private double targetPosition = 0.5;
    private final PIDController positionPID;

    /**
     * Constructor for TurretSystem.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public TurretSystem(HardwareMap hardwareMap) {
        try {
            turretServo = hardwareMap.get(CRServo.class, "turretSer");
            turretEncoder = hardwareMap.get(AnalogInput.class, "turretEnc");

            // Try to get turret motor if using encoder-based positioning
            try {
                turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
                if (turretMotor != null) {
                    turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            } catch (Exception e) {
                // Motor not available, use servo only
                turretMotor = null;
            }

            // FIXED: Initialize PID controller for CRServo position control
            positionPID = new PIDController(KP_POSITION, KI_POSITION, KD_POSITION);

            encoderExecutorService = Executors.newSingleThreadExecutor();
            encoderExecutorService.submit(this::readEncoderContinuously);

            // FIXED: Initialize servo to stopped state
            turretServo.setPower(0);
        } catch (Exception e) {
            throw new RuntimeException("Turret system initialization failed", e);
        }
    }

    /**
     * Continuously reads the turret encoder value on a background thread.
     */
    private void readEncoderContinuously() {
        while (!Thread.currentThread().isInterrupted()) {
            if (turretEncoder != null) {
                currentPosition = Math.max(0.0, Math.min(1.0, turretEncoder.getVoltage() / 3.3));
                // Convert position to angle (assuming 0.5 = center = 0 degrees)
                currentAngleDegrees = (currentPosition - 0.5) * 140.0; // ±70 degrees range
            } else if (turretMotor != null) {
                // Use motor encoder for angle tracking
                int position = turretMotor.getCurrentPosition();
                currentAngleDegrees = position / TICKS_PER_DEGREE;
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
     * Main update loop for the turret.
     * FIXED: Added PID position control for CRServo
     */
    @Override
    public void update() {
        if (currentMode == ControlMode.POSITION_CONTROL) {
            // Use PID to calculate required power to reach target position
            double power = positionPID.calculate(currentPosition, targetPosition);

            // Clamp power to safe limits
            power = Math.max(-MAX_POSITION_POWER, Math.min(MAX_POSITION_POWER, power));

            // If we're close enough to target, stop (prevent oscillation)
            if (Math.abs(currentPosition - targetPosition) < POSITION_TOLERANCE) {
                power = 0;
            }

            turretServo.setPower(power);
        }
        // In MANUAL mode, power is set directly via setPower() - no periodic logic needed
    }

    @Override
    public void stop() {
        setPower(0);
        if (encoderExecutorService != null) {
            encoderExecutorService.shutdownNow();
            try {
                if (!encoderExecutorService.awaitTermination(SHUTDOWN_TIMEOUT_MS, TimeUnit.MILLISECONDS)) {
                    System.err.println("Turret encoder thread did not terminate gracefully.");
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Turret Position", "%.3f", currentPosition);
        telemetry.addData("Turret Angle", "%.1f°", currentAngleDegrees);
        telemetry.addData("Turret Mode", currentMode);
        if (currentMode == ControlMode.POSITION_CONTROL) {
            telemetry.addData("Turret Target", "%.1f°", (targetPosition - 0.5) * 140.0);
            telemetry.addData("Turret Error", "%.1f°", Math.abs(currentAngleDegrees - (targetPosition - 0.5) * 140.0));
        } else {
            telemetry.addData("Turret Power", turretServo.getPower());
        }
    }

    /**
     * Sets the raw power of the turret servo for manual control.
     * Switches to manual mode.
     *
     * @param power The power to apply (-1.0 to 1.0).
     */
    public void setPower(double power) {
        currentMode = ControlMode.MANUAL;
        positionPID.reset();  // Reset PID when switching to manual
        turretServo.setPower(power * MANUAL_POWER_MULTIPLIER);
    }

    // ==================================================
    // V I S I O N   T A R G E T I N G   M E T H O D S
    // ==================================================

    /**
     * Gets the current turret angle in degrees.
     * 0 = center, positive = right, negative = left
     *
     * @return Current angle in degrees
     */
    public double getCurrentAngleDegrees() {
        return currentAngleDegrees;
    }

    /**
     * Sets a target angle for the turret to rotate to.
     * FIXED: Now uses proper PID control for CRServo + encoder setup
     * Automatically handles limits (±70 degrees).
     *
     * @param angleDegrees Target angle in degrees (0 = center)
     */
    public void setTargetAngle(double angleDegrees) {
        // Clamp to limits
        angleDegrees = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, angleDegrees));

        if (turretMotor != null) {
            // Use motor-based position control (existing code)
            int targetPosition = (int)(angleDegrees * TICKS_PER_DEGREE);
            turretMotor.setTargetPosition(targetPosition);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(0.5);
            currentMode = ControlMode.MANUAL; // Motor handles its own control
        } else {
            // FIXED: Use PID control for CRServo + encoder
            // Convert angle to position (0.0 to 1.0, where 0.5 = center)
            targetPosition = (angleDegrees / 140.0) + 0.5;
            targetPosition = Math.max(0.0, Math.min(1.0, targetPosition));

            // Switch to position control mode
            currentMode = ControlMode.POSITION_CONTROL;
            positionPID.reset();  // Reset PID for new target
        }
    }

    /**
     * Checks if the turret is currently moving to a target position.
     *
     * @return true if turret is busy moving
     */
    public boolean isBusy() {
        if (turretMotor != null) {
            return turretMotor.isBusy();
        }

        // FIXED: For CRServo, check if we're in position control and not at target
        if (currentMode == ControlMode.POSITION_CONTROL) {
            return Math.abs(currentPosition - targetPosition) > POSITION_TOLERANCE;
        }

        // In manual mode, not busy
        return false;
    }

    /**
     * Checks if turret is at the target position.
     *
     * @return true if at target (within tolerance)
     */
    public boolean isAtTarget() {
        if (currentMode == ControlMode.POSITION_CONTROL) {
            return Math.abs(currentPosition - targetPosition) <= POSITION_TOLERANCE;
        }
        return true; // In manual mode, always "at target"
    }

    /**
     * Centers the turret (0 degrees).
     */
    public void center() {
        setTargetAngle(0.0);
    }

    // ==================================================
    // G E T T E R S
    // ==================================================

    public double getPosition() {
        return currentPosition;
    }

    public ControlMode getControlMode() {
        return currentMode;
    }
}