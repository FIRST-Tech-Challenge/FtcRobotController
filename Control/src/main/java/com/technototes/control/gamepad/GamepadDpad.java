package com.technototes.control.gamepad;

/** A class for dpads
 * @author Alex Stedman
 * @param <T> The gamepad button class
 */
public class GamepadDpad<T extends GamepadButton> implements Stick {
    public T up, down, left, right;

    /** Create dpad with 4 buttons
     *
     * @param u Up button
     * @param d Down button
     * @param l Left button
     * @param r Right button
     */
    public GamepadDpad(T u, T d, T l, T r) {
        up = u;
        down = d;
        left = l;
        right = r;
    }

    /** Return x axis double (treating dpad as stick)
     *
     * @return The double
     */
    @Override
    public double getXAxis() {
        return (right.getAsBoolean() ? (left.getAsBoolean() ? 0 : 1) : (left.getAsBoolean() ? -1 : 0));
    }
    /** Return y axis double (treating dpad as stick)
     *
     * @return The double
     */
    @Override
    public double getYAxis() {
        return (up.getAsBoolean() ? (down.getAsBoolean() ? 0 : 1) : (down.getAsBoolean() ? -1 : 0));
    }

    @Override
    public void periodic() {
        up.periodic();
        down.periodic();
        left.periodic();
        right.periodic();
    }
}
