package com.technototes.library.subsystem.motor;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.subsystem.SpeedSubsystem;

/** Class for motor subsystems
 * @author Alex Stedman
 * @param <T> The motor type
 */
public class MotorSubsystem<T extends Motor<?>> extends Subsystem<T> implements SpeedSubsystem {
    /** Create motor subsystem
     *
     * @param motors The motors
     */
    @SafeVarargs
    public MotorSubsystem(T... motors) {
        super(motors);
    }

    /** Get the speed of the motors in the subsystem
     *
     * @return The speed
     */
    @Override
    public double getSpeed() {
        return getDevices()[0].getSpeed();
    }

    /** Set the speed of the primary motors in subsystem
     *
     * @param speed The speed
     */
    @Override
    public void setSpeed(double speed) {
        for (T m : getDevices()) {
            m.setSpeed(speed);
        }
    }
}
