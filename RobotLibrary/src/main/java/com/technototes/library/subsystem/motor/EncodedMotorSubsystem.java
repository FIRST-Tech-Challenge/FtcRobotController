package com.technototes.library.subsystem.motor;

import com.technototes.library.hardware.motor.EncodedMotor;

/** Class for encoded motor subsystems
 * @author Alex Stedman
 */
public class EncodedMotorSubsystem extends MotorSubsystem<EncodedMotor<?>>{
    /** Max speed
     *
     */
    public double maxSpeed = 0.5;

    /** Create encoded motor subsystem
     *
     * @param motors The motors
     */
    public EncodedMotorSubsystem(EncodedMotor<?>... motors) {
        super(motors);
    }

    /** Set the max speed for the subsystem
     *
     * @param speed New max speed
     * @return this
     */
    public EncodedMotorSubsystem setMaxSpeed(double speed) {
        maxSpeed = speed;
        return this;
    }

    /** Set position for subsystem with existing max speed
     *
     * @param ticks Motor ticks for position
     * @return If this is at specified position
     */
    public boolean setPosition(double ticks) {
        return setPosition(ticks, maxSpeed);
    }
    /** Set position for subsystem
     *
     * @param ticks Motor ticks for position
     * @param speed The max speed to run to the position
     * @return If this is at specified position
     */
    public boolean setPosition(double ticks, double speed) {
        boolean b = true;
        for (EncodedMotor<?> s : getDevices()) {
            if (!s.setPosition(ticks, speed))
                b = false;
        }
        return b;
    }


}
