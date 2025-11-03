package org.firstinspires.ftc.teamcode.Mechanisms.utils;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Manages limit switch functionality with debouncing and state tracking
 */
public class LimitSwitchManager {
    private final DigitalChannel limitSwitch;
    private boolean lastState = false;
    private boolean currentState = false;
    private final ElapsedTime debounceTimer = new ElapsedTime();
    private static final double DEBOUNCE_TIME_MS = 50;

    public LimitSwitchManager(DigitalChannel limitSwitch) {
        this.limitSwitch = limitSwitch;
        if (limitSwitch != null) {
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }
        debounceTimer.reset();
    }

    public void update() {
        if (limitSwitch == null) return;

        boolean rawState = !limitSwitch.getState(); // Inverted because typically pressed = low

        // Debouncing logic
        if (debounceTimer.milliseconds() > DEBOUNCE_TIME_MS) {
            currentState = rawState;
            debounceTimer.reset();
        }

        lastState = currentState;
    }

    public boolean isPressed() {
        return currentState;
    }

    public boolean wasPressed() {
        return lastState;
    }

    public boolean wasReleased() {
        return !currentState && lastState;
    }

    public boolean wasPressedThisCycle() {
        return currentState && !lastState;
    }

    public boolean isConnected() {
        return limitSwitch != null;
    }
}