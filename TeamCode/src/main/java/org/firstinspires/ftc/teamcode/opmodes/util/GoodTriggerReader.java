package org.firstinspires.ftc.teamcode.opmodes.util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;

/**
 * Reason: original bad
 */
public class GoodTriggerReader implements KeyReader {

    private GamepadEx gamepad;

    private GamepadKeys.Trigger trigger;

    /**
     * Last state of the button
     **/
    private boolean lastState;

    /**
     * Current state of the button
     **/
    private boolean currState;

    /**
     * Initializes controller variables
     *
     * @param gamepad The controller joystick
     * @param trigger The controller button
     **/
    public GoodTriggerReader(GamepadEx gamepad, GamepadKeys.Trigger trigger) {
        this.gamepad = gamepad;
        this.trigger = trigger;

        if (this.gamepad.getTrigger(trigger) > 0.5) {
            currState = true;
        } else {
            currState = false;
        }

        lastState = currState;
    }

    /**
     * Reads button value
     **/
    public void readValue() {
        lastState = currState;
        if (this.gamepad.getTrigger(trigger) > 0.5) {
            currState = true;
        } else {
            currState = false;
        }
    }

    /**
     * Checks if the button is down
     **/
    public boolean isDown() {
        return currState;
    }

    /**
     * Checks if the button was just pressed
     **/
    public boolean wasJustPressed() {
        return (!lastState && currState);
    }

    /**
     * Checks if the button was just released
     **/
    public boolean wasJustReleased() {
        return (lastState && !currState);
    }

    /**
     * Checks if the button state has changed
     **/
    public boolean stateJustChanged() {
        return (lastState != currState);
    }

}
