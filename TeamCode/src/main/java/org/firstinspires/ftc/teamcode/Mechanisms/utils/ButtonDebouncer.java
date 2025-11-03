package org.firstinspires.ftc.teamcode.Mechanisms.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Debounces button inputs to prevent multiple triggers from single presses
 */
public class ButtonDebouncer {
    private boolean lastState = false;
    private boolean currentState = false;
    private final ElapsedTime debounceTimer = new ElapsedTime();
    private static final double DEBOUNCE_TIME_MS = 50;

    public ButtonDebouncer() {
        debounceTimer.reset();
    }

    public void update(boolean currentRawState) {
        // Debouncing logic
        if (debounceTimer.milliseconds() > DEBOUNCE_TIME_MS) {
            lastState = currentState;
            currentState = currentRawState;
            debounceTimer.reset();
        }
    }

    public boolean isPressed() {
        return currentState;
    }

    public boolean wasPressed() {
        return lastState;
    }

    public boolean wasPressedThisCycle() {
        return currentState && !lastState;
    }

    public boolean wasReleased() {
        return !currentState && lastState;
    }
}