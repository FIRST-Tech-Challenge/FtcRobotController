package org.firstinspires.ftc.teamcode.ultimategoal.util;

/**
 * A class that contains all the logic needed for a button to be a toggle, meaning it can be held
 * and will not return true continuously. (it will only return true once per press)
 */
public class ToggleButton {
    private boolean toggle = false;

    /**
     * Determines if a button has been toggled, by keeping track of whether or not the button
     * press has changed since the last call of this method.
     *
     * @param buttonInput The raw input of the button, e.g. whether or not it is pressed.
     * @return true on toggle, false if not.
     */
    public boolean isToggled(boolean buttonInput) {
        if (!toggle && buttonInput) {
            toggle = true;
            return true;
        } else if (toggle && !buttonInput) {
            toggle = false;
        }

        return false;
    }

    /**
     * Determines if a trigger has been toggled, by keeping track of whether or not the trigger
     * press has changed since the last call of this method. A trigger pressed down any amount
     * is treated as a button that is pressed.
     *
     * @param triggerInput The raw input of the trigger, which is how far down it is pushed. 0
     *                     means it is not being pushed at all.
     * @return true on toggle, false if not.
     */
    public boolean isToggled(double triggerInput) {
        boolean isPressed = triggerInput != 0;

        return isToggled(isPressed);
    }
}
