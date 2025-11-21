package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.LimitSwitchManager;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.SensorManager;

/**
 * The SpindexerSystem manages the rotating chamber that holds and positions rings for the launcher.
 * It uses a state machine to handle homing, rotating to specific steps, and automatic ball loading.
 */
public class SpindexerSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final double TICKS_PER_STEP = 377.67;
    private static final double HOMING_POWER = 0.4;
    private static final double MOVING_POWER = 0.5;
    private static final double BALL_DETECTION_DISTANCE_MM = 50.0;
    private static final int MAX_BALLS = 3;
    private static final long HOMING_TIMEOUT_MS = 5000;
    private static final long MOVE_TO_STEP_TIMEOUT_MS = 3000;

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final DcMotorEx spindexerMotor;
    private final LimitSwitchManager limitSwitch;
    private final SensorManager ballSensor;

    // ==================================================
    // S T A T E   M A C H I N E
    // ==================================================
    private enum State { IDLE, HOMING, MOVING_TO_STEP, MANUAL_CONTROL }
    private State currentState = State.IDLE;
    private int targetStep = 0;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // ==================================================
    // S T A T E
    // ==================================================
    private int ballsLoaded = 0;
    private boolean autoIntakeEnabled = true;

    /**
     * Constructor for SpindexerSystem.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public SpindexerSystem(HardwareMap hardwareMap) {
        try {
            spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexer");
            spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            limitSwitch = new LimitSwitchManager(hardwareMap, "limit");
            ballSensor = new SensorManager(hardwareMap, "colorSensor", BALL_DETECTION_DISTANCE_MM);
        } catch (Exception e) {
            throw new RuntimeException("Spindexer system initialization failed", e);
        }
    }

    /**
     * Main update loop for the spindexer.
     */
    @Override
    public void update() {
        switch (currentState) {
            case IDLE:
                if (autoIntakeEnabled && ballSensor.isBallDetected() && ballsLoaded < MAX_BALLS) {
                    ballsLoaded++;
                    increment();
                }
                break;
            case HOMING:
                if (stateTimer.milliseconds() > HOMING_TIMEOUT_MS || limitSwitch.isPressed()) {
                    if (limitSwitch.isPressed()) {
                        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    setState(State.IDLE);
                }
                break;
            case MOVING_TO_STEP:
                if (stateTimer.milliseconds() > MOVE_TO_STEP_TIMEOUT_MS || !spindexerMotor.isBusy()) {
                    setState(State.IDLE);
                }
                break;
            case MANUAL_CONTROL:
                // Manual power is set directly, no periodic logic needed.
                break;
        }
    }

    /**
     * Initiates the homing sequence.
     */
    public void home() {
        if (isBusy()) return;
        setState(State.HOMING);
    }

    /**
     * Increments the spindexer to the next step.
     */
    public void increment() {
        if (isBusy()) return;
        int nextStep = (getCurrentStep() + 1) % MAX_BALLS;
        moveToStep(nextStep);
    }

    /**
     * Moves the spindexer to a specific step.
     *
     * @param step The target step (0, 1, or 2).
     */
    public void moveToStep(int step) {
        if (isBusy()) return;
        if (step == 0) {
            home();
        } else {
            targetStep = step;
            setState(State.MOVING_TO_STEP);
        }
    }

    /**
     * Sets raw power to the spindexer motor for manual control.
     *
     * @param power The power to apply (-1.0 to 1.0).
     */
    public void setManualPower(double power) {
        setState(State.MANUAL_CONTROL);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setPower(power);
    }

    @Override
    public void stop() {
        setState(State.IDLE);
        limitSwitch.stop();
        ballSensor.stop();
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Spindexer Step", getCurrentStep() + " / " + (MAX_BALLS - 1));
        telemetry.addData("Spindexer Balls Loaded", ballsLoaded + " / " + MAX_BALLS);
        telemetry.addData("Spindexer State", currentState);
        telemetry.addData("Spindexer Auto-Intake", autoIntakeEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Spindexer Limit Switch", limitSwitch.isPressed() ? "PRESSED" : "Released");
        telemetry.addData("Spindexer Ball Sensor", ballSensor.isBallDetected() ? "DETECTED" : "NONE");
    }

    /**
     * Changes the internal state of the spindexer and configures motors accordingly.
     */
    private void setState(State newState) {
        if (currentState == newState) return;
        currentState = newState;
        stateTimer.reset();

        switch (newState) {
            case IDLE:
            case MANUAL_CONTROL:
                spindexerMotor.setPower(0);
                spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case HOMING:
                spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                spindexerMotor.setPower(HOMING_POWER);
                break;
            case MOVING_TO_STEP:
                int targetPos = (int) (targetStep * TICKS_PER_STEP);
                spindexerMotor.setTargetPosition(targetPos);
                spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindexerMotor.setPower(MOVING_POWER);
                break;
        }
    }

    // ==================================================
    // G E T T E R S   A N D   S E T T E R S
    // ==================================================

    public int getCurrentStep() {
        return (int) Math.round(spindexerMotor.getCurrentPosition() / TICKS_PER_STEP);
    }

    public boolean isBusy() {
        return currentState == State.HOMING || currentState == State.MOVING_TO_STEP;
    }

    public int getBallsLoaded() {
        return ballsLoaded;
    }

    public void setBallsLoaded(int count) {
        this.ballsLoaded = Math.max(0, Math.min(MAX_BALLS, count));
    }

    public void setAutoIntakeEnabled(boolean enabled) {
        this.autoIntakeEnabled = enabled;
    }
}
