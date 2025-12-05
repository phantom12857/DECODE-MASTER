// UPDATED: SpindexerSystem.java  
package org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.core.Subsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.GamePiece;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.LimitSwitchManager;
import org.firstinspires.ftc.teamcode.Mechanisms.utils.SensorManager;

import java.util.Arrays;

/**
 * The SpindexerSystem now tracks the color of each game piece in its slots
 * and supports an auto-indexing feature.
 * <p>
 * REFACTORED: Added setBallsLoaded() method and enhanced state management
 */
public class SpindexerSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final double TICKS_PER_STEP = 377.67;
    private static final double HOMING_POWER = 0.4;
    private static final double MOVING_POWER = 0.5;
    private static final double BALL_DETECTION_DISTANCE_MM = 50.0;
    private static final int MAX_SLOTS = 3;
    private static final long HOMING_TIMEOUT_MS = 5000;
    private static final long MOVE_TO_STEP_TIMEOUT_MS = 3000;
    private static final long INTAKE_HOLD_TIME_MS = 250;

    // ==================================================
    // H A R D W A R E
    // ==================================================
    private final DcMotorEx spindexerMotor;
    private final LimitSwitchManager limitSwitch;
    private final SensorManager ballSensor;

    // ==================================================
    // S T A T E   M A C H I N E
    // ==================================================
    private enum State { IDLE, HOMING, MOVING_TO_STEP, MANUAL_CONTROL, ERROR }
    private State currentState = State.IDLE;
    private State previousState = State.IDLE;
    private int targetStep = 0;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private String lastError = "";

    // ==================================================
    // S T A T E
    // ==================================================
    private final GamePiece.Color[] slots = new GamePiece.Color[MAX_SLOTS];
    private boolean autoIndexEnabled = true; // Auto-indexing is on by default
    private boolean isHoldingForIntake = false;
    private int ballsLoaded = 0;  // Track number of balls loaded

    /**
     * Constructor for SpindexerSystem.
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
            Arrays.fill(slots, GamePiece.Color.NONE);
        } catch (Exception e) {
            throw new RuntimeException("Spindexer system initialization failed", e);
        }
    }

    @Override
    public void update() {
        // Error state handling
        if (currentState == State.ERROR) {
            // Stay in error state until manually cleared
            return;
        }

        // The core of the auto-indexing logic.
        if (autoIndexEnabled && currentState == State.IDLE && !isHoldingForIntake) {
            GamePiece.Color detectedColor = ballSensor.getDetectedColor();
            if (detectedColor != GamePiece.Color.NONE) {
                // A ball has been detected at the intake.
                int currentStep = getCurrentStep();
                if (slots[currentStep] == GamePiece.Color.NONE) {
                    // The current slot is empty, so load the ball.
                    slots[currentStep] = detectedColor;
                    ballsLoaded++;
                    // Hold here briefly to allow the intake to fully seat the piece before indexing.
                    isHoldingForIntake = true;
                    stateTimer.reset();
                }
            }
        }

        // After holding for a moment, advance to the next empty slot.
        if (isHoldingForIntake && stateTimer.milliseconds() > INTAKE_HOLD_TIME_MS) {
            isHoldingForIntake = false;
            // We find the next empty slot to avoid overwriting a loaded piece.
            int nextEmptyStep = findNextEmptyStep();
            if (nextEmptyStep != -1) {
                moveToStep(nextEmptyStep);
            }
        }

        // State machine for motor control.
        switch (currentState) {
            case HOMING:
                if (stateTimer.milliseconds() > HOMING_TIMEOUT_MS) {
                    // Homing timed out - this is an error condition
                    handleError("Homing timeout - limit switch not reached");
                    setState(State.ERROR);
                } else if (limitSwitch.isPressed()) {
                    // Successfully reached home position
                    spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Arrays.fill(slots, GamePiece.Color.NONE); // Homing resets everything
                    ballsLoaded = 0;
                    setState(State.IDLE);
                }
                break;

            case MOVING_TO_STEP:
                if (stateTimer.milliseconds() > MOVE_TO_STEP_TIMEOUT_MS) {
                    // Movement timed out - motor may be stalled
                    handleError("Move to step timeout - motor may be stalled");
                    setState(State.ERROR);
                } else if (!spindexerMotor.isBusy()) {
                    // Successfully reached target position
                    setState(State.IDLE);
                }
                break;

            default:
                break;
        }
    }

    /**
     * Starts homing sequence to find the home position using limit switch.
     */
    public void home() {
        if (isBusy()) {
            System.err.println("Spindexer: Cannot home while busy");
            return;
        }
        setState(State.HOMING);
    }

    /**
     * Advances the spindexer to the next position.
     */
    public void increment() {
        if (isBusy()) {
            System.err.println("Spindexer: Cannot increment while busy");
            return;
        }
        int nextStep = (getCurrentStep() + 1) % MAX_SLOTS;
        moveToStep(nextStep);
    }

    /**
     * Moves the spindexer to a specific step position.
     *
     * @param step The target step (0-2)
     */
    public void moveToStep(int step) {
        if (isBusy()) {
            System.err.println("Spindexer: Cannot move to step while busy");
            return;
        }
        if (step < 0 || step >= MAX_SLOTS) {
            System.err.println("Spindexer: Invalid step " + step);
            return;
        }
        targetStep = step;
        setState(State.MOVING_TO_STEP);
    }

    /**
     * Finds the next empty slot in the spindexer.
     *
     * @return The index of the next empty slot, or -1 if all slots are full
     */
    private int findNextEmptyStep() {
        int current = getCurrentStep();
        for (int i = 1; i <= MAX_SLOTS; i++) {
            int nextStep = (current + i) % MAX_SLOTS;
            if (slots[nextStep] == GamePiece.Color.NONE) {
                return nextStep;
            }
        }
        return -1; // All slots are full
    }

    /**
     * Transitions to a new state with appropriate motor control.
     *
     * @param newState The state to transition to
     */
    private void setState(State newState) {
        if (currentState == newState) return;

        previousState = currentState;
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

            case ERROR:
                spindexerMotor.setPower(0);
                spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                System.err.println("Spindexer entered ERROR state: " + lastError);
                break;
        }
    }

    /**
     * Handles error conditions by recording error message.
     *
     * @param errorMessage Description of the error
     */
    private void handleError(String errorMessage) {
        lastError = errorMessage;
        System.err.println("Spindexer Error: " + errorMessage);
    }

    /**
     * Clears error state and returns to idle.
     * Call this after resolving the error condition.
     */
    public void clearError() {
        if (currentState == State.ERROR) {
            lastError = "";
            setState(State.IDLE);
        }
    }

    @Override
    public void stop() {
        setState(State.IDLE);
        if (limitSwitch != null) limitSwitch.stop();
        if (ballSensor != null) ballSensor.stop();
    }

    @Override
    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Spindexer State", currentState);
        telemetry.addData("Spindexer Position", "Step %d/%d", getCurrentStep(), MAX_SLOTS - 1);
        telemetry.addData("Spindexer Slots", Arrays.toString(slots));
        telemetry.addData("Spindexer Balls Loaded", "%d/%d", ballsLoaded, MAX_SLOTS);
        telemetry.addData("Spindexer Auto-Index", autoIndexEnabled ? "ENABLED" : "DISABLED");

        if (currentState == State.ERROR) {
            telemetry.addData("⚠️ Spindexer ERROR", lastError);
        }
    }

    // ==================================================
    // G E T T E R S   A N D   S E T T E R S
    // ==================================================

    /**
     * Gets the current step position of the spindexer.
     *
     * @return Current step (0-2)
     */
    public int getCurrentStep() {
        return (int) Math.round(spindexerMotor.getCurrentPosition() / TICKS_PER_STEP) % MAX_SLOTS;
    }

    /**
     * Clears a specific slot, marking it as empty.
     *
     * @param slotIndex The slot to clear (0-2)
     */
    public void clearSlot(int slotIndex) {
        if (slotIndex >= 0 && slotIndex < MAX_SLOTS) {
            if (slots[slotIndex] != GamePiece.Color.NONE) {
                ballsLoaded--;
            }
            slots[slotIndex] = GamePiece.Color.NONE;
        }
    }

    /**
     * Sets the number of balls loaded and updates slot states.
     * FIXED: This method was missing but called in TeleOpController line 130
     *
     * @param count Number of balls to set (typically 0 to clear all)
     */
    public void setBallsLoaded(int count) {
        ballsLoaded = Math.max(0, Math.min(count, MAX_SLOTS));

        if (count == 0) {
            // Clear all slots
            Arrays.fill(slots, GamePiece.Color.NONE);
        } else {
            // Set first 'count' slots to generic loaded state
            for (int i = 0; i < MAX_SLOTS; i++) {
                if (i < count) {
                    // Mark as loaded but unknown color
                    if (slots[i] == GamePiece.Color.NONE) {
                        slots[i] = GamePiece.Color.GREEN; // Default color
                    }
                } else {
                    slots[i] = GamePiece.Color.NONE;
                }
            }
        }
    }

    /**
     * Gets the number of balls currently loaded.
     *
     * @return Ball count (0-3)
     */
    public int getBallsLoaded() {
        return ballsLoaded;
    }

    /**
     * Checks if the spindexer is busy (homing or moving).
     *
     * @return true if busy, false if idle
     */
    public boolean isBusy() {
        return currentState == State.HOMING || currentState == State.MOVING_TO_STEP;
    }

    /**
     * Checks if the spindexer is in an error state.
     *
     * @return true if in error state
     */
    public boolean isInError() {
        return currentState == State.ERROR;
    }

    /**
     * Gets the last error message.
     *
     * @return Error message, or empty string if no error
     */
    public String getLastError() {
        return lastError;
    }

    /**
     * Toggles the auto-indexing feature on/off.
     */
    public void toggleAutoIndex() {
        autoIndexEnabled = !autoIndexEnabled;
    }

    /**
     * Sets the auto-indexing feature.
     *
     * @param enabled true to enable, false to disable
     */
    public void setAutoIndexEnabled(boolean enabled) {
        autoIndexEnabled = enabled;
    }

    /**
     * Checks if auto-indexing is enabled.
     *
     * @return true if enabled
     */
    public boolean isAutoIndexEnabled() {
        return autoIndexEnabled;
    }

    /**
     * Gets a copy of the current slot colors.
     * Used by MechanismCoordinator for intelligent firing sequences.
     *
     * @return Array of 3 colors representing current slot states
     */
    public GamePiece.Color[] getSlotColors() {
        return Arrays.copyOf(slots, slots.length);
    }

    /**
     * Gets the color of a specific slot.
     *
     * @param slotIndex Slot index (0-2)
     * @return Color in that slot, or NONE if invalid index
     */
    public GamePiece.Color getSlotColor(int slotIndex) {
        if (slotIndex >= 0 && slotIndex < MAX_SLOTS) {
            return slots[slotIndex];
        }
        return GamePiece.Color.NONE;
    }

    /**
     * Manually sets the color of a slot.
     * Useful for testing or manual calibration.
     *
     * @param slotIndex Slot index (0-2)
     * @param color Color to set
     */
    public void setSlotColor(int slotIndex, GamePiece.Color color) {
        if (slotIndex >= 0 && slotIndex < MAX_SLOTS) {
            // Update slot color
            boolean wasEmpty = (slots[slotIndex] == GamePiece.Color.NONE);
            boolean isEmptyNow = (color == GamePiece.Color.NONE);

            slots[slotIndex] = color;

            // Update ball count
            if (wasEmpty && !isEmptyNow) {
                ballsLoaded++;
            } else if (!wasEmpty && isEmptyNow) {
                ballsLoaded--;
            }
        }
    }
}