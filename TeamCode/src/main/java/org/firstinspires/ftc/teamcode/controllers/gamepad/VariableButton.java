package org.firstinspires.ftc.teamcode.controllers.gamepad;

public abstract class VariableButton extends Input{
    private static final float PRESS_THRESHOLD = 0.85f;
    /** Abstract method which runs every loop
     * @param value   if this button is being pressed
     */
    public abstract void run(boolean value);

    /** Internal method used by GamepadEx class
     * @return if this button is currently pressed
     */
    protected abstract float detect();

    @Override
    protected void updateInput() {
        run(detect() > PRESS_THRESHOLD);
    }
}
