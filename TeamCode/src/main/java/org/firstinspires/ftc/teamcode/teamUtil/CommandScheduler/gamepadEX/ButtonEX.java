package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX;

public class ButtonEX {
    private Boolean internalInput = false;
    private Boolean previousState = false;
    private double leadingEdgeDebounce = 0;
    private double trailingEdgeDebounce = 0;
    private long lastCheck = System.nanoTime() - 100;

    private Boolean processedInput = false;

    public ButtonEX() {}

    /**
     * must be called at the end of every teleop loop, or other environment in which the {@link GamepadEX} inputs are used, which should be handled by {@link GamepadEX} update method
     */
    public void endLoopUpdate() {
        previousState = internalInput;
    }

    public void startLoopUpdate(boolean gamepadInput){
        internalInput = gamepadInput;
    }

    /**
     * includes debouncing
     *
     * @return returns the boolean value of the button with debouncing used
     */
    public Boolean isPressed() {
        if (internalInput != previousState) {
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

    public Boolean raw() {
        return internalInput;
    }

    public Boolean onPress() {
        return isPressed() && isPressed() != previousState;
    }

    public Boolean onRelease() {
        return !isPressed() && isPressed() != previousState;
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
    public ButtonEX debounce(debouncingType type, double duration) {
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
}
