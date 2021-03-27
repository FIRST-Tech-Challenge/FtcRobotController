package com.technototes.library.hardware.servo;

import com.technototes.library.hardware.*;
import com.technototes.logger.Log;
import com.technototes.logger.Stated;

/** Class for servos
 * @author Alex Stedman
 */
public class Servo extends HardwareDevice<com.qualcomm.robotcore.hardware.Servo> implements Sensored, Invertable<Servo>, Followable<Servo> {

    //public double pid_p, pid_i, pid_d;

    /** Create servo object
     *
     * @param device The servo
     */
    public Servo(com.qualcomm.robotcore.hardware.Servo device) {
        super(device);
    }

    /** Create servo object
     *
     * @param deviceName The device name in hardware map
     */
    public Servo(String deviceName) {
        super(deviceName);
    }

    /** Set position for the servo and return this
     *
     * @param position The servo position
     * @return this
     */
    public Servo setStartingPosition(double position) {
        setPosition(position);
        return this;
    }

    @Override
    public boolean getInverted() {
        return getDevice().getDirection() == com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
    }

    @Override
    public Servo setInverted(boolean invert) {
        getDevice().setDirection(invert ? com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD : com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        return this;
    }

    /** Set servo position
     *
     * @param position The position to set the servo to
     */
    public void setPosition(double position) {
        getDevice().setPosition(position);
    }

    @Log
    @Override
    public double getSensorValue() {
        return getDevice().getPosition();
    }

    /** Get servo position
     *
     * @return The servo position
     */
    public double getPosition(){
        return getSensorValue();
    }

    /** Set servo range
     *
     * @param min The minimum of the range
     * @param max The maximum of the range
     * @return this
     */
    public Servo setRange(double min, double max) {
        getDevice().scaleRange(min, max);
        return this;
    }
    @Deprecated
    @Override
    public Servo follow(Servo d) {
        return new ServoGroup(this, d);
    }

//    @Override
//    public void setPIDValues(double p, double i, double d) {
//        pid_p = p;
//        pid_i = i;
//        pid_d = d;
//    }

//    @Override
//    public boolean setPositionPID(double val) {
//        device.setPosition(PIDUtils.calculatePIDDouble(pid_p, pid_i, pid_d, device.getPosition(), val));
//        return isAtPosition(val);
//    }

}
