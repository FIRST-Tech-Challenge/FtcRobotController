package org.firstinspires.ftc.teamcode.controllers.gamepad;

public abstract class VariableToggle extends Input{
    private boolean lastState = false;
    private boolean toggleState = false;
    private static final float PRESS_THRESHOLD = 0.85f;

    /** Abstract method which when toggled
     * @param value   Current toggled value of this input
     */
    public abstract void onToggle(boolean value);

    /** Internal method used by GamepadEx class
     * @return if this button is currently pressed
     */
    protected abstract float detect();

    @Override
    protected void updateInput() {
        boolean currentState = detect() > PRESS_THRESHOLD;
        if (currentState != lastState) {
            lastState = currentState;
            if (currentState) {
                toggleState = !toggleState;
                onToggle(toggleState);
            }
        }
    }
}
