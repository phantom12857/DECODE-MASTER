// UPDATED: TurretSystem.java
package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * The TurretSystem manages the rotating turret for aiming the launcher.
 * It provides manual control and background reading of its position.
 *
 * REFACTORED: Added angle-based control methods for vision targeting
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
    private double currentAngleDegrees = 0.0;  // Only accessed from main thread

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

            encoderExecutorService = Executors.newSingleThreadExecutor();
            encoderExecutorService.submit(this::readEncoderContinuously);
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
     */
    @Override
    public void update() {
        // Turret is controlled directly by setPower() or setTargetAngle(), 
        // no periodic logic needed for basic operation
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
        telemetry.addData("Turret Power", turretServo.getPower());
    }

    /**
     * Sets the raw power of the turret servo for manual control.
     *
     * @param power The power to apply (-1.0 to 1.0).
     */
    public void setPower(double power) {
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
     * Automatically handles limits (±70 degrees).
     *
     * @param angleDegrees Target angle in degrees (0 = center)
     */
    public void setTargetAngle(double angleDegrees) {
        // Clamp to limits
        angleDegrees = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, angleDegrees));

        if (turretMotor != null) {
            // Convert degrees to encoder ticks
            int targetPosition = (int)(angleDegrees * TICKS_PER_DEGREE);
            turretMotor.setTargetPosition(targetPosition);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(0.5);  // Adjust power as needed
        } else {
            // Fallback for servo-based control
            // Convert angle to position (0.0 to 1.0)
            double targetPosition = (angleDegrees / 140.0) + 0.5;
            // This is a simplified approach - you may need more sophisticated control
            double error = targetPosition - currentPosition;
            double power = Math.max(-1.0, Math.min(1.0, error * 2.0));
            setPower(power);
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
        // For servo-based control, assume not busy after a short delay
        // You may want to implement more sophisticated busy detection
        return Math.abs(turretServo.getPower()) > 0.1;
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
}