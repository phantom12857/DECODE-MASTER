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
 * The LauncherSystem manages the flywheel and kicker for launching rings.
 * It uses a PID controller to maintain flywheel speed and a state machine for the kicker action.
 */
public class LauncherSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final double MAX_RPM = 5000.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double KICKER_EXTEND_POS = 0.67;
    private static final double KICKER_RETRACT_POS = 1.0;
    private static final long KICK_DURATION_MS = 500;
    private static final long THREAD_SLEEP_MS = 50;
    private static final long SHUTDOWN_TIMEOUT_MS = 100;

    // PIDF Coefficients
    private static final double KP = 0.02, KI = 0.001, KD = 0.005, KF = 0.0;

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final DcMotorEx launcherMotor;
    private final Servo kickerServo;

    // ==================================================
    // S T A T E
    // ==================================================
    private final PIDController pidController;
    private final ExecutorService velocityExecutorService;
    private volatile double currentRPM = 0.0;
    private double targetRPM = 0.0;

    private enum KickerState { RETRACTED, KICKING }
    private KickerState kickerState = KickerState.RETRACTED;
    private final ElapsedTime kickerTimer = new ElapsedTime();

    /**
     * Constructor for LauncherSystem.
     *
     * @param hardwareMap The robot's hardware map.
     */
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

    /**
     * Continuously reads the launcher motor velocity on a background thread.
     */
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

    /**
     * Main update loop for the launcher.
     */
    @Override
    public void update() {
        updateFlywheelPID();
        updateKickerState();
    }

    /**
     * Manages the PID control for the flywheel motor.
     */
    private void updateFlywheelPID() {
        if (targetRPM > 0) {
            double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;
            double correction = pidController.calculate(launcherMotor.getVelocity(), targetTicksPerSec);
            double power = (targetTicksPerSec + correction) / ((MAX_RPM / 60.0) * TICKS_PER_REV); // Normalize power
            launcherMotor.setPower(power + KF);
        } else {
            launcherMotor.setPower(0);
        }
    }

    /**
     * Manages the state of the kicker servo.
     */
    private void updateKickerState() {
        if (kickerState == KickerState.KICKING && kickerTimer.milliseconds() > KICK_DURATION_MS) {
            kickerServo.setPosition(KICKER_RETRACT_POS);
            kickerState = KickerState.RETRACTED;
        }
    }

    /**
     * Initiates the kick sequence if all conditions are met.
     */
    public void kick() {
        if (kickerState == KickerState.RETRACTED && isAtSpeed(100)) {
            kickerServo.setPosition(KICKER_EXTEND_POS);
            kickerState = KickerState.KICKING;
            kickerTimer.reset();
        }
    }

    @Override
    public void stop() {
        setRPM(0);
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
        telemetry.addData("Launcher Target RPM", "%.0f", targetRPM);
        telemetry.addData("Launcher Actual RPM", "%.0f", currentRPM);
        telemetry.addData("Launcher At Speed", isAtSpeed(100));
        telemetry.addData("Kicker State", kickerState);
    }

    // ==================================================
    // G E T T E R S   A N D   S E T T E R S
    // ==================================================

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
        return Math.abs(currentRPM - targetRPM) <= toleranceRPM && targetRPM > 0;
    }
}
