package org.firstinspires.ftc.teamcode.Mechanisms.utils;

/**
 * The ButtonDebouncer class provides a simple way to handle digital button presses
 * and releases, accounting for the mechanical and electrical noise (bouncing)
 * that can occur when a button is pressed.
 */
public class ButtonDebouncer {

    // ==================================================
    // S T A T E
    // ==================================================
    private boolean lastState = false;
    private boolean currentState = false;

    /**
     * Updates the state of the button.
     * This method should be called once per loop with the raw button state.
     *
     * @param rawButtonState The current, real-time state of the button (true for pressed).
     */
    public void update(boolean rawButtonState) {
        lastState = currentState;
        currentState = rawButtonState;
    }

    /**
     * Checks if the button is currently pressed.
     *
     * @return True if the button is pressed, false otherwise.
     */
    public boolean isPressed() {
        return currentState;
    }

    /**
     * Checks if the button was just pressed in the current cycle.
     * This is a rising-edge detector.
     *
     * @return True only on the single loop cycle that the button transitions from released to pressed.
     */
    public boolean wasPressedThisCycle() {
        return currentState && !lastState;
    }

    /**
     * Checks if the button was just released in the current cycle.
     * This is a falling-edge detector.
     *
     * @return True only on the single loop cycle that the button transitions from pressed to released.
     */
    public boolean wasReleasedThisCycle() {
        return !currentState && lastState;
    }
}
