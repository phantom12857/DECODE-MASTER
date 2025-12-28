// FIXED: SpindexerSystem.java with proper rotation and auto-indexing
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
 * FIXED: SpindexerSystem with corrected rotation logic and enhanced auto-indexing
 *
 * FIXES:
 * 1. Proper modulo handling for negative positions
 * 2. Shortest-path rotation (always forward)
 * 3. Enhanced auto-indexing with color sensor
 * 4. Auto-advance after ball fired
 */
public class SpindexerSystem implements Subsystem {

    // ==================================================
    // C O N S T A N T S
    // ==================================================
    private static final double TICKS_PER_STEP = 377.67;  // Encoder ticks for 120° rotation
    private static final double HOMING_POWER = 0.4;
    private static final double MOVING_POWER = 0.6;  // INCREASED: Was 0.5, now faster
    private static final double BALL_DETECTION_DISTANCE_MM = 60.0;  // TUNED: Was 50, now more sensitive
    private static final int MAX_SLOTS = 3;
    private static final long HOMING_TIMEOUT_MS = 5000;
    private static final long MOVE_TO_STEP_TIMEOUT_MS = 3000;
    private static final long INTAKE_HOLD_TIME_MS = 400;  // INCREASED: Was 250, now holds longer

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
    private boolean autoIndexEnabled = true; // Auto-indexing on by default
    private boolean isHoldingForIntake = false;
    private int ballsLoaded = 0;
    private boolean autoAdvanceAfterFire = true;  // NEW: Auto-advance after firing

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
            return;
        }

        // ENHANCED AUTO-INDEXING: Detects balls and advances to next empty slot
        if (autoIndexEnabled && currentState == State.IDLE && !isHoldingForIntake) {
            GamePiece.Color detectedColor = ballSensor.getDetectedColor();
            if (detectedColor != GamePiece.Color.NONE) {
                // Ball detected at intake position
                int currentStep = getCurrentStep();
                if (slots[currentStep] == GamePiece.Color.NONE) {
                    // Load ball into current slot
                    slots[currentStep] = detectedColor;
                    ballsLoaded++;
                    // Hold briefly to let intake fully seat the ball
                    isHoldingForIntake = true;
                    stateTimer.reset();
                }
            }
        }

        // After holding, advance to next empty slot
        if (isHoldingForIntake && stateTimer.milliseconds() > INTAKE_HOLD_TIME_MS) {
            isHoldingForIntake = false;
            int nextEmptyStep = findNextEmptyStep();
            if (nextEmptyStep != -1) {
                moveToStep(nextEmptyStep);
            }
        }

        // State machine for motor control
        switch (currentState) {
            case HOMING:
                if (stateTimer.milliseconds() > HOMING_TIMEOUT_MS) {
                    handleError("Homing timeout - limit switch not reached");
                    setState(State.ERROR);
                } else if (limitSwitch.isPressed()) {
                    // Successfully reached home
                    spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Arrays.fill(slots, GamePiece.Color.NONE);
                    ballsLoaded = 0;
                    setState(State.IDLE);
                }
                break;

            case MOVING_TO_STEP:
                if (stateTimer.milliseconds() > MOVE_TO_STEP_TIMEOUT_MS) {
                    handleError("Move to step timeout - motor may be stalled");
                    setState(State.ERROR);
                } else if (!spindexerMotor.isBusy()) {
                    // Reached target position
                    setState(State.IDLE);
                }
                break;

            default:
                break;
        }
    }

    /**
     * Starts homing sequence to find home position using limit switch.
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
     * FIXED: Moves spindexer to specific step using shortest path.
     * Always rotates FORWARD (no backwards rotation).
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
        return -1; // All slots full
    }

    /**
     * FIXED: Transitions to new state with corrected rotation logic.
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
                // FIXED: Calculate shortest forward path
                int currentPos = spindexerMotor.getCurrentPosition();
                int currentStep = getCurrentStep();

                // Calculate steps to move forward (always positive)
                int stepsToMove = (targetStep - currentStep + MAX_SLOTS) % MAX_SLOTS;

                // Calculate target position (always move forward)
                int targetPos = currentPos + (int)(stepsToMove * TICKS_PER_STEP);

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
     * Handles error conditions.
     */
    private void handleError(String errorMessage) {
        lastError = errorMessage;
        System.err.println("Spindexer Error: " + errorMessage);
    }

    /**
     * Clears error state and returns to idle.
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
        telemetry.addData("Spindexer Balls", "%d/%d", ballsLoaded, MAX_SLOTS);
        telemetry.addData("Auto-Index", autoIndexEnabled ? "ON" : "OFF");
        telemetry.addData("Auto-Advance", autoAdvanceAfterFire ? "ON" : "OFF");

        if (currentState == State.ERROR) {
            telemetry.addData("⚠️ ERROR", lastError);
        }

        // DIAGNOSTIC: Show encoder position
        telemetry.addData("Encoder Pos", spindexerMotor.getCurrentPosition());
    }

    // ==================================================
    // G E T T E R S   A N D   S E T T E R S
    // ==================================================

    /**
     * FIXED: Gets current step with proper negative number handling.
     */
    public int getCurrentStep() {
        int position = spindexerMotor.getCurrentPosition();
        int step = (int) Math.round(position / TICKS_PER_STEP);

        // FIXED: Handle negative positions properly
        step = step % MAX_SLOTS;
        if (step < 0) step += MAX_SLOTS;  // Convert negative to positive

        return step;
    }

    /**
     * Clears a specific slot.
     */
    public void clearSlot(int slotIndex) {
        if (slotIndex >= 0 && slotIndex < MAX_SLOTS) {
            if (slots[slotIndex] != GamePiece.Color.NONE) {
                ballsLoaded--;
            }
            slots[slotIndex] = GamePiece.Color.NONE;

            // NEW: Auto-advance to next slot after firing if enabled
            if (autoAdvanceAfterFire && !isBusy()) {
                int nextSlot = (slotIndex + 1) % MAX_SLOTS;
                moveToStep(nextSlot);
            }
        }
    }

    /**
     * Sets number of balls loaded.
     */
    public void setBallsLoaded(int count) {
        ballsLoaded = Math.max(0, Math.min(count, MAX_SLOTS));

        if (count == 0) {
            Arrays.fill(slots, GamePiece.Color.NONE);
        } else {
            for (int i = 0; i < MAX_SLOTS; i++) {
                if (i < count) {
                    if (slots[i] == GamePiece.Color.NONE) {
                        slots[i] = GamePiece.Color.GREEN; // Default
                    }
                } else {
                    slots[i] = GamePiece.Color.NONE;
                }
            }
        }
    }

    public int getBallsLoaded() {
        return ballsLoaded;
    }

    public boolean isBusy() {
        return currentState == State.HOMING || currentState == State.MOVING_TO_STEP;
    }

    public boolean isInError() {
        return currentState == State.ERROR;
    }

    public String getLastError() {
        return lastError;
    }

    public void toggleAutoIndex() {
        autoIndexEnabled = !autoIndexEnabled;
    }

    public void setAutoIndexEnabled(boolean enabled) {
        autoIndexEnabled = enabled;
    }

    public boolean isAutoIndexEnabled() {
        return autoIndexEnabled;
    }

    /**
     * NEW: Enable/disable auto-advance after firing.
     */
    public void setAutoAdvanceAfterFire(boolean enabled) {
        autoAdvanceAfterFire = enabled;
    }

    public boolean isAutoAdvanceEnabled() {
        return autoAdvanceAfterFire;
    }

    public GamePiece.Color[] getSlotColors() {
        return Arrays.copyOf(slots, slots.length);
    }

    public GamePiece.Color getSlotColor(int slotIndex) {
        if (slotIndex >= 0 && slotIndex < MAX_SLOTS) {
            return slots[slotIndex];
        }
        return GamePiece.Color.NONE;
    }

    public void setSlotColor(int slotIndex, GamePiece.Color color) {
        if (slotIndex >= 0 && slotIndex < MAX_SLOTS) {
            boolean wasEmpty = (slots[slotIndex] == GamePiece.Color.NONE);
            boolean isEmptyNow = (color == GamePiece.Color.NONE);

            slots[slotIndex] = color;

            if (wasEmpty && !isEmptyNow) {
                ballsLoaded++;
            } else if (!wasEmpty && isEmptyNow) {
                ballsLoaded--;
            }
        }
    }
}