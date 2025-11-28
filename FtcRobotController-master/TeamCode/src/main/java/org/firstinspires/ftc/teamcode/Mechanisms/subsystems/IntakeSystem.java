package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;

/**
 * The IntakeSystem class manages the robot's intake mechanism.
 * It provides simple state-based control for running the intake forward, backward, or stopping it.
 */
public class IntakeSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final double INTAKE_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;
    private static final double PASSIVE_POWER = 0.2; // Small power to hold pixels

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final DcMotor intakeMotor;

    // ==================================================
    // S T A T E   M A C H I N E
    // ==================================================
    public enum State { INTAKING, REVERSING, PASSIVE, STOPPED }
    private State currentState = State.STOPPED;

    /**
     * Constructor for IntakeSystem.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public IntakeSystem(HardwareMap hardwareMap) {
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            throw new RuntimeException("Intake system initialization failed", e);
        }
    }

    /**
     * Sets the intake to run forward.
     */
    public void start() {
        setState(State.INTAKING);
    }

    /**
     * Sets the intake to run in reverse.
     */
    public void reverse() {
        setState(State.REVERSING);
    }

    /**
     * Sets the intake to a low power to gently hold pixels.
     */
    public void passiveIntake() {
        setState(State.PASSIVE);
    }

    /**
     * Stops the intake motor.
     */
    @Override
    public void stop() {
        setState(State.STOPPED);
    }

    /**
     * Updates the motor power based on the current state.
     * This method is called from the main robot loop.
     */
    @Override
    public void update() {
        switch (currentState) {
            case INTAKING:
                intakeMotor.setPower(INTAKE_POWER);
                break;
            case REVERSING:
                intakeMotor.setPower(REVERSE_POWER);
                break;
            case PASSIVE:
                intakeMotor.setPower(PASSIVE_POWER);
                break;
            case STOPPED:
            default:
                intakeMotor.setPower(0.0);
                break;
        }
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Intake State", currentState);
        telemetry.addData("Intake Power", intakeMotor.getPower());
    }

    /**
     * Changes the internal state of the intake.
     *
     * @param newState The new state to transition to.
     */
    private void setState(State newState) {
        this.currentState = newState;
    }

    /**
     * Gets the current state of the intake.
     *
     * @return The current IntakeSystem.State.
     */
    public State getState() {
        return currentState;
    }
}
