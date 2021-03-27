package com.technototes.library.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.technototes.library.hardware.Sensored;
import com.technototes.library.hardware.sensor.encoder.Encoder;
import com.technototes.library.hardware.sensor.encoder.ExternalEncoder;
import com.technototes.library.hardware.sensor.encoder.MotorEncoder;
import com.technototes.logger.Log;
import com.technototes.logger.Stated;

/** Class for encoded motors
 * @author Alex Stedman
 * @param <T> The qualcomm motor device interface
 */
public class EncodedMotor<T extends DcMotorSimple> extends Motor<T> implements Sensored {

    //public PIDCoefficients coefficients;
    /** Deadzone for going to positions with encoder
     *
     */
    public double positionThreshold = 50;

    private Encoder encoder;

    /** Make encoded motor
     *
     * @param device The dcmotor object
     */
    public EncodedMotor(T device) {
        super(device);
        //coefficients = new PIDCoefficients(0, 0, 0);
        if (device instanceof DcMotor) {
            encoder = new MotorEncoder((DcMotor) device);
        }
    }

    public EncodedMotor<T> setEncoder(Encoder enc){
        encoder = enc;
        return this;
    }

    /** Make encoded motor
     *
     * @param deviceName The dcmotor device name in hardware map
     */
    public EncodedMotor(String deviceName) {
        super(deviceName);
        //coefficients = new PIDCoefficients(0, 0, 0);
        //controller = new PIDFController(coefficients);
        //coefficients = new PIDCoefficients(0, 0, 0);
        if (getDevice() instanceof DcMotor) {
            encoder = new MotorEncoder((DcMotor) getDevice());
        }
    }

//    public EncodedMotor setPID(double p, double i, double d){
//        coefficients = new PIDCoefficients(p, i, d);
//        return this;
//    }

    @Override
    public EncodedMotor<T> setInverted(boolean invert) {
        getDevice().setDirection(invert ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        return this;
    }

    @Override
    public EncodedMotor<T> invert() {
        return setInverted(!getInverted());
    }

    @Override
    public double getSensorValue() {
        return encoder.getSensorValue();
    }
//
//    @Override
//    public void setPIDValues(double p, double i, double d) {
//        coefficients = new PIDCoefficients(p, i, d);
//    }

//    @Override
//    public boolean setPositionPID(double val) {
//        if (!isAtPosition(val)) {
//            setSpeed(MathUtils.constrain(-0.1,(val-getSensorValue())/(coefficients.kP*10000), 0.1)*10);
//        } else {
//            setSpeed(0);
//            return true;
//        }
//        return false;
//    }

    /** Set the position of the motor
     *
     * @param ticks The encoder ticks
     * @return Is the motor at this position
     */
    public boolean setPosition(double ticks) {
        return setPosition(ticks, 0.5);
    }
    /** Set the position of the motor
     *
     * @param ticks The encoder ticks
     * @param speed The speed to run the motor at
     * @return Is the motor at this position
     */
    public boolean setPosition(double ticks, double speed) {
        if (!isAtPosition(ticks)) {
            setSpeed(getSensorValue() < ticks ? speed : -speed);
        } else {
            setSpeed(0);
            return true;
        }
        return false;
    }

    /** Is the motor at the specified position
     *
     * @param ticks The position
     * @return Is the motor here or not
     */
    public boolean isAtPosition(double ticks) {
        return Math.abs(ticks - getSensorValue()) < positionThreshold;
    }

    /** Get the encoder object
     *
     * @return The encoder
     */
    public Encoder getEncoder(){
        return encoder;
    }

    /** Zero the encoder
     * @return This
     */
    public EncodedMotor<T> zeroEncoder(){
        encoder.zeroEncoder();
        return this;
    }

    @Override
    public Double getState() {
        return getSensorValue();
    }

    /** Set velocity of motor in tps
     *
     * @param tps the speed in encoder ticks per second
     */
    public void setVelocity(double tps) {
        if(getDevice() instanceof DcMotor){
            ((DcMotor) getDevice()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            getDevice().setPower(tps);
        } else {
            //TODO velo for non dcmotors
        }
    }

    public double getVelocity(){
        if(getDevice() instanceof DcMotor){
            ((DcMotor) getDevice()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return getDevice().getPower();
        } else {
            //TODO velo for non dcmotors
            return 0;
        }
    }
    @Override
    public double getSpeed(){
        if(getDevice() instanceof DcMotor){
            ((DcMotor) getDevice()).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return getDevice().getPower();
        } else {
            //TODO velo for non dcmotors
            return 0;
        }
    }


    @Override
    public void setSpeed(double speed) {
        if(getDevice() instanceof DcMotor) ((DcMotor) getDevice()).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.setSpeed(speed);
    }
}
