package com.technototes.control.gamepad;

import com.technototes.control.Periodic;

/** A class for gamepad sticks
 * @author Alex Stedman
 * @param <T> The class for the gamepad axis
 * @param <U> The class for the gamepad buttons
 */
public class GamepadStick<T extends GamepadAxis, U extends GamepadButton> implements Stick {
    public T xAxis, yAxis;
    public U stickButton;

    /** Make a gamepad stick
     *
     * @param x The x joystick axis
     * @param y The y joystick axis
     * @param b The joystick button
     */
    public GamepadStick(T x, T y, U b){
        xAxis = x;
        yAxis = y;
        stickButton = b;
    }

    @Override
    public void periodic() {
        xAxis.periodic();
        yAxis.periodic();
        stickButton.periodic();
    }
    /** Return x axis double
     *
     * @return The double
     */
    @Override
    public double getXAxis() {
        return xAxis.getAsDouble();
    }
    /** Return y axis double
     *
     * @return The double
     */
    @Override
    public double getYAxis() {
        return yAxis.getAsDouble();
    }
}
