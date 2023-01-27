package org.firstinspires.ftc.teamcode.teamUtil.gamepadEX;

import com.qualcomm.robotcore.hardware.Gamepad;

public class button {
    private final gamepadEX.buttons button;
    private final Gamepad gamepad;
    private boolean internalInput;
    private boolean previousState = false;
    private double leadingEdgeDebounce = 0;
    private double trailingEdgeDebounce = 0;
    private long lastCheck = -1000000;

    private boolean processedInput = false;

    button(gamepadEX.buttons button, Gamepad gamepad) {
        this.button = button;
        this.gamepad = gamepad;
        internalInput = getInput();
    }

    /**
     * must be called at the end of every teleop loop, or other environment in which the {@link gamepadEX} inputs are used, which should be handled by {@link gamepadEX} update method
     */
    void update() {
        previousState = internalInput;
    }

    /**
     * includes debouncing
     *
     * @return returns the boolean value of the button with debouncing used
     */
    public boolean isPressed() {
        if (raw() != previousState) {
            lastCheck = System.nanoTime();
        }
        if (internalInput && System.nanoTime() - lastCheck >= leadingEdgeDebounce) {
            processedInput = true;
            lastCheck = System.nanoTime();
        }
        if (!internalInput && System.nanoTime() - lastCheck >= trailingEdgeDebounce) {
            processedInput = false;
            lastCheck = System.nanoTime();
        }

        return processedInput;
    }

    public boolean raw() {
        //internalInput = getInput(); TODO test if updating the value of internalInput every loop is required
        return getInput();
    }

    public boolean onPress() {
        return isPressed() && isPressed() != previousState;
    }

    public boolean onRelease() {
        return !isPressed() && isPressed() != previousState;
    }

    public void actionOnPress(buttonAction buttonAction) {
        if(onRelease()){
            buttonAction.buttonAction();
        }
    }

    public void actionOnRelease(buttonAction buttonAction) {
        if(onPress()){
            buttonAction.buttonAction();
        }
    }

    public void actionWhilePressed(buttonAction buttonAction) {
        if(isPressed()){
            buttonAction.buttonAction();
        }
    }

    public void actionWhileReleased(buttonAction buttonAction) {
        if(!isPressed()){
            buttonAction.buttonAction();
        }
    }
    public interface buttonAction{
        void buttonAction();
    }

    public enum debouncingType {
        LEADING_EDGE,
        TRAILING_EDGE,
        BOTH
    }

    /**
     * applies debouncing to the button
     *
     * @param type     type of debouncing to be updated, leading edge effects the change from true to false, trailing edge effects the change from false to true
     * @param duration the duration of the debouncing to be applied, in seconds
     * @return returns the updated button object
     */
    public button debounce(debouncingType type, double duration) {
        switch (type) {
            case LEADING_EDGE:
                leadingEdgeDebounce = duration * 1E9;
                break;
            case TRAILING_EDGE:
                trailingEdgeDebounce = duration * 1E9;
                break;
            case BOTH:
                leadingEdgeDebounce = duration * 1E9;
                trailingEdgeDebounce = duration * 1E9;
                break;
        }
        return this;
    }

    private boolean getInput() {
        switch (this.button) {
            case a:
                return gamepad.a;
            case b:
                return gamepad.b;
            case x:
                return gamepad.x;
            case y:
                return gamepad.y;
            case dpad_up:
                return gamepad.dpad_up;
            case dpad_down:
                return gamepad.dpad_down;
            case dpad_left:
                return gamepad.dpad_left;
            case dpad_right:
                return gamepad.dpad_right;
            case right_bumper:
                return gamepad.right_bumper;
            case left_bumper:
                return gamepad.left_bumper;
        }
        return false;
    }
}
