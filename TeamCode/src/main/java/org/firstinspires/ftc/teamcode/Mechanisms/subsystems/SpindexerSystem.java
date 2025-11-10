package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.LimitSwitchManager;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.SensorManager;

public class SpindexerSystem implements Subsystem {

    private final DcMotor spindexer;
    private final LimitSwitchManager limitSwitchManager;
    private final SensorManager sensorManager;

    private int currentStep = 0;
    public int ballsLoaded = 0;
    public boolean autoIntakeEnabled = true;

    private enum State { IDLE, HOMING, MOVING_TO_STEP }
    private State state = State.IDLE;
    private int targetStep = 0;
    private final ElapsedTime timeoutTimer = new ElapsedTime();
    private final ElapsedTime ballCheckTimer = new ElapsedTime();

    private static final double TICKS_PER_STEP = 350;
    private static final double HOMING_POWER = 0.4;
    private static final double MOVING_POWER = 0.5;

    public SpindexerSystem(HardwareMap hardwareMap) {
        try {
            spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");

            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spindexer.setDirection(DcMotorSimple.Direction.REVERSE);
            spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Initialize managers
            limitSwitchManager = new LimitSwitchManager(
                    hardwareMap.get(DigitalChannel.class, "limit")
            );

            sensorManager = new SensorManager(
                    hardwareMap.get(ColorSensor.class, "colorSensor"),
                    hardwareMap.get(DistanceSensor.class, "colorSensor")
            );

            ballCheckTimer.reset();
        } catch (Exception e) {
            throw new RuntimeException("Spindexer system initialization failed", e);
        }
    }

    @Override
    public void update() {
        // Update managers
        limitSwitchManager.update();
        sensorManager.update();

        switch (state) {
            case IDLE:
                break;

            case HOMING:
                if (timeoutTimer.seconds() > 5.0 || limitSwitchManager.isPressed()) {
                    spindexer.setPower(0);
                    if (limitSwitchManager.isPressed()) {
                        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        currentStep = 0;
                    }
                    state = State.IDLE;
                }
                break;

            case MOVING_TO_STEP:
                if (timeoutTimer.seconds() > 3.0 || !spindexer.isBusy()) {
                    spindexer.setPower(0);
                    spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (!spindexer.isBusy()) {
                        currentStep = targetStep;
                    }
                    state = State.IDLE;
                }
                break;
        }

        // Auto ball detection
        if (autoIntakeEnabled && !isMoving() && ballCheckTimer.seconds() > 0.2) {
            checkForBall();
            ballCheckTimer.reset();
        }
    }

    private void checkForBall() {
        if (ballsLoaded >= 3 || !sensorManager.isBallDetected()) return;

        ballsLoaded++;
        increment();
    }

    public void home() {
        if (isMoving()) return;

        state = State.HOMING;
        timeoutTimer.reset();

        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setPower(HOMING_POWER);
    }

    public void increment() {
        if (isMoving()) return;
        int nextStep = (currentStep + 1) % 3;
        moveToStep(nextStep);
    }

    public void moveToStep(int step) {
        if (step == 0) {
            home();
        } else {
            state = State.MOVING_TO_STEP;
            targetStep = step;
            timeoutTimer.reset();

            int targetPos = (int)(step * TICKS_PER_STEP);

            spindexer.setTargetPosition(targetPos);
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexer.setPower(MOVING_POWER);
        }
    }

    @Override
    public void stop() {
        if (spindexer != null) {
            spindexer.setPower(0);
        }
        state = State.IDLE;
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Spindexer", "Step %d, Balls: %d/3, Moving: %b",
                currentStep, ballsLoaded, isMoving());
        telemetry.addData("Spindexer State", state);
        telemetry.addData("Limit Switch", limitSwitchManager.isPressed() ? "PRESSED" : "Released");
        telemetry.addData("Auto-Intake", autoIntakeEnabled ? "ENABLED" : "DISABLED");

        if (sensorManager.isConnected()) {
            telemetry.addData("Ball Detected", sensorManager.isBallDetected() ? "YES" : "NO");
            telemetry.addData("Distance", "%.1f mm", sensorManager.getBallDistance());
            telemetry.addData("Color", "R:%d G:%d B:%d",
                    sensorManager.getRed(), sensorManager.getGreen(), sensorManager.getBlue());
        }
    }

    // Getters
    public int getCurrentStep() { return currentStep; }
    public boolean isMoving() { return state != State.IDLE; }
    public boolean isLimitPressed() { return limitSwitchManager.isPressed(); }
    public int getBallsLoaded() { return ballsLoaded; }
    public void setBallsLoaded(int count) { ballsLoaded = Math.max(0, Math.min(3, count)); }
    public void setAutoIntakeEnabled(boolean enabled) { autoIntakeEnabled = enabled; }
}