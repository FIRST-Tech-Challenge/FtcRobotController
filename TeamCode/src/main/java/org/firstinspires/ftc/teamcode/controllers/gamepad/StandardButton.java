package org.firstinspires.ftc.teamcode.controllers.gamepad;

public abstract class StandardButton extends Input {
    /** Abstract method which runs every loop
     * @param value   If this button is being pressed
     */
    public abstract void run(boolean value);

    /** Internal method used by GamepadEx class
     * @return if this button is currently pressed
     */
    protected abstract boolean detect();

    @Override
    protected void updateInput() {
        run(detect());
    }
}
