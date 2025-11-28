// UPDATED: LauncherSystem.java
package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.PIDController;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * Enhanced LauncherSystem with faster spin-up between shots.
 * Uses idle speed to keep flywheel spinning and distance-based RPM.
 */
public class LauncherSystem implements Subsystem {

    private static final double MAX_RPM = 5000.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double KICKER_EXTEND_POS = 0.67;
    private static final double KICKER_RETRACT_POS = 1.0;
    private static final long KICK_DURATION_MS = 500;
    private static final long THREAD_SLEEP_MS = 50;
    private static final long SHUTDOWN_TIMEOUT_MS = 100;

    // Enhanced PID with feed-forward for faster response
    private static final double KP = 0.03, KI = 0.002, KD = 0.008, KF = 0.12;

    // Idle speed keeps flywheel spinning between shots for faster recovery
    private static final double IDLE_RPM = 2500.0;  // Minimum spin between shots
    private static final double RPM_TOLERANCE = 100.0;

    private final DcMotorEx launcherMotor;
    private final Servo kickerServo;

    private final PIDController pidController;
    private final ExecutorService velocityExecutorService;
    private volatile double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private boolean maintainIdleSpeed = true;  // Keep spinning between shots

    private enum KickerState { RETRACTED, KICKING }
    private KickerState kickerState = KickerState.RETRACTED;
    private final ElapsedTime kickerTimer = new ElapsedTime();

    public LauncherSystem(HardwareMap hardwareMap) {
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
            kickerServo = hardwareMap.get(Servo.class, "kicker");

            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            kickerServo.setPosition(KICKER_RETRACT_POS);

            pidController = new PIDController(KP, KI, KD);

            velocityExecutorService = Executors.newSingleThreadExecutor();
            velocityExecutorService.submit(this::readLauncherVelocity);
        } catch (Exception e) {
            throw new RuntimeException("Launcher system initialization failed", e);
        }
    }

    private void readLauncherVelocity() {
        while (!Thread.currentThread().isInterrupted()) {
            currentRPM = (launcherMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            try {
                Thread.sleep(THREAD_SLEEP_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    @Override
    public void update() {
        updateFlywheelPID();
        updateKickerState();
    }

    private void updateFlywheelPID() {
        double effectiveTarget = targetRPM;

        // If no target set and idle mode enabled, maintain idle speed
        if (targetRPM <= 0 && maintainIdleSpeed) {
            effectiveTarget = IDLE_RPM;
        }

        if (effectiveTarget > 0) {
            double targetTicksPerSec = (effectiveTarget / 60.0) * TICKS_PER_REV;
            double correction = pidController.calculate(launcherMotor.getVelocity(), targetTicksPerSec);

            // Feed-forward: estimate base power needed for target RPM
            double feedForward = (effectiveTarget / MAX_RPM) * KF;

            // Normalize PID correction
            double pidPower = correction / ((MAX_RPM / 60.0) * TICKS_PER_REV);

            // Combine feed-forward and PID
            double totalPower = feedForward + pidPower;
            totalPower = Math.max(0, Math.min(1.0, totalPower));

            launcherMotor.setPower(totalPower);
        } else {
            launcherMotor.setPower(0);
        }
    }

    private void updateKickerState() {
        if (kickerState == KickerState.KICKING && kickerTimer.milliseconds() > KICK_DURATION_MS) {
            kickerServo.setPosition(KICKER_RETRACT_POS);
            kickerState = KickerState.RETRACTED;
        }
    }

    /**
     * Sets target RPM based on distance using linear regression.
     * This is called automatically by MechanismCoordinator when using vision.
     */
    public void setRPMForDistance(double distanceInches) {
        // Linear regression: RPM = 12.5 * distance + 2800
        double rpm = (12.5 * distanceInches) + 2800;
        setRPM(Math.max(2800, Math.min(4500, rpm)));
    }

    public void kick() {
        // Allow kicking at lower tolerance for faster cycling
        if (kickerState == KickerState.RETRACTED && isAtSpeed(150)) {
            kickerServo.setPosition(KICKER_EXTEND_POS);
            kickerState = KickerState.KICKING;
            kickerTimer.reset();
        }
    }

    /**
     * Forces kick even if not quite at speed (emergency use).
     */
    public void forceKick() {
        if (kickerState == KickerState.RETRACTED) {
            kickerServo.setPosition(KICKER_EXTEND_POS);
            kickerState = KickerState.KICKING;
            kickerTimer.reset();
        }
    }

    @Override
    public void stop() {
        setRPM(0);
        maintainIdleSpeed = false;
        launcherMotor.setPower(0);
        if (velocityExecutorService != null) {
            velocityExecutorService.shutdownNow();
            try {
                if (!velocityExecutorService.awaitTermination(SHUTDOWN_TIMEOUT_MS, TimeUnit.MILLISECONDS)) {
                    System.err.println("Launcher velocity thread did not terminate.");
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Launcher Target", "%.0f RPM", targetRPM > 0 ? targetRPM : IDLE_RPM);
        telemetry.addData("Launcher Actual", "%.0f RPM", currentRPM);
        telemetry.addData("Launcher Error", "%.0f RPM", Math.abs(currentRPM - (targetRPM > 0 ? targetRPM : IDLE_RPM)));
        telemetry.addData("Launcher At Speed", isAtSpeed(100));
        telemetry.addData("Kicker", kickerState);
    }

    public void setRPM(double rpm) {
        this.targetRPM = Math.min(rpm, MAX_RPM);
        this.pidController.reset();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getActualRPM() {
        return currentRPM;
    }

    public boolean isKicking() {
        return kickerState != KickerState.RETRACTED;
    }

    public boolean isAtSpeed(double toleranceRPM) {
        double effectiveTarget = targetRPM > 0 ? targetRPM : (maintainIdleSpeed ? IDLE_RPM : 0);
        return Math.abs(currentRPM - effectiveTarget) <= toleranceRPM && effectiveTarget > 0;
    }

    /**
     * Enable or disable idle speed mode.
     * When enabled, flywheel spins at IDLE_RPM between shots for faster recovery.
     */
    public void setMaintainIdleSpeed(boolean maintain) {
        this.maintainIdleSpeed = maintain;
    }

    public boolean isIdleSpeedEnabled() {
        return maintainIdleSpeed;
    }
}