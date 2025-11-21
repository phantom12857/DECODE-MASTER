package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * The TurretSystem manages the rotating turret for aiming the launcher.
 * It provides manual control and background reading of its position.
 */
public class TurretSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final double MANUAL_POWER_MULTIPLIER = 0.8;
    private static final long THREAD_SLEEP_MS = 50;
    private static final long SHUTDOWN_TIMEOUT_MS = 100;

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final CRServo turretServo;
    private final AnalogInput turretEncoder;

    // ==================================================
    // S T A T E
    // ==================================================
    private final ExecutorService encoderExecutorService;
    private volatile double currentPosition = 0.5;

    /**
     * Constructor for TurretSystem.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public TurretSystem(HardwareMap hardwareMap) {
        try {
            turretServo = hardwareMap.get(CRServo.class, "turretSer");
            turretEncoder = hardwareMap.get(AnalogInput.class, "turretEnc");

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
            currentPosition = turretEncoder != null ? Math.max(0.0, Math.min(1.0, turretEncoder.getVoltage() / 3.3)) : 0.5;
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
        // Turret is controlled directly by setPower(), no periodic logic needed.
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
    // G E T T E R S
    // ==================================================

    public double getPosition() {
        return currentPosition;
    }
}
