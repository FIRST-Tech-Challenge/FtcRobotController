package org.firstinspires.ftc.teamcode.controllers.gamepad;

public abstract class ToggleButton extends Input {
    private boolean lastValue = false;
    private boolean toggledValue = false;

    /** Abstract method which runs when the button is pressed
     * @param value   The toggled state of this button
     */
    public abstract void onToggle(boolean value);

    /** Internal method used by GamepadEx class
     * @return if this button is currently pressed
     */
    protected abstract boolean detect();

    @Override
    protected void updateInput() {
        boolean currentState = detect();
        if (currentState != lastValue) {
            lastValue = currentState;
            if (currentState) {
                toggledValue = !toggledValue;
                onToggle(toggledValue);
            }
        }
    }
}
