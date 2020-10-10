package com.technototes.control.gamepad;

import com.technototes.control.Periodic;

import java.util.function.BooleanSupplier;

/** The class to extend custom gamepad buttons from
 * @author Alex Stedman
 */
public class GamepadButton implements BooleanSupplier, Periodic {
    private BooleanSupplier booleanSupplier;
    public boolean pressed = false;
    public boolean toggle = false;
    public boolean recentAction = false;
    public boolean pastState = false;

    /** Create button with boolean supplier
     *
     * @param b The supplier
     */
    public GamepadButton(BooleanSupplier b){
        booleanSupplier = b;
    }
    public GamepadButton(){
        booleanSupplier = () -> false;
    }
    /** Set the boolean supplier for the button
     *
     * @param b The supplier
     * @return This
     */
    public GamepadButton setSupplier(BooleanSupplier b){
        booleanSupplier = b;
        return this;
    }

    @Override
    public void periodic(){
        periodic(booleanSupplier.getAsBoolean());
    }

    public void periodic(boolean currentState){
        recentAction = pastState != currentState;
        pastState = currentState;
        pressed = currentState;
        toggle = (recentAction && pastState) ? !toggle : toggle;
    }

    /** Returns if the button is just pressed
     *
     * @return The above condition
     */
    public boolean isJustActivated(){
        return pressed && recentAction;
    }
    /** Returns if the button is just released
     *
     * @return The above condition
     */
    public boolean isJustDeactivated(){
        return !pressed && recentAction;
    }
    /** Returns if the button is pressed
     *
     * @return The above condition
     */
    public boolean isActivated(){
        return pressed;
    }
    /** Returns if the button is released
     *
     * @return The above condition
     */
    public boolean isDeactivated(){
        return !pressed;
    }
    /** Returns if the button is just toggled
     *
     * @return The above condition
     */
    public boolean isJustToggled(){
        return toggle && recentAction && pressed;
    }
    /** Returns if the button is just untoggled
     *
     * @return The above condition
     */
    public boolean isJustInverseToggled(){
        return !toggle && recentAction && pressed;
    }
    /** Returns if the button is toggled
     *
     * @return The above condition
     */
    public boolean isToggled(){
        return toggle;
    }
    /** Returns if the button is untoggled
     *
     * @return The above condition
     */
    public boolean isInverseToggled(){
        return !toggle;
    }

    /** Returns the button state as boolean (is the exact same as isActivated())
     *
     * @return The above condition
     */
    @Override
    public boolean getAsBoolean() {
        return isActivated();
    }
}
