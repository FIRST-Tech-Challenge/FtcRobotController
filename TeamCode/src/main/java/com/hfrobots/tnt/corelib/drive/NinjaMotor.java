/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.corelib.drive;

import android.util.Log;

import com.hfrobots.tnt.corelib.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * A DcMotor wrapper implementing our ExtendedDcMotor interface that
 * adds required functionality (such as knowing the encoder counts). Factory methods
 * exist for the motor models/brands we use that allow programmers to "forget" which
 * way the motors rotate and how many PPR each motor has.
 */
public class NinjaMotor implements ExtendedDcMotor {

    private final double encoderCountsPerRevolution;
    private final DcMotor dcMotor;
    private int absoluteTargetPosition;
    private final Rotation motorNativeDirection;
    private int zeroEncoderValue;
    private int prevEncoderPosition;
    private MotorVelocityTracker velocityTracker;

    public static ExtendedDcMotor wrap32Motor(DcMotor dcMotor) {
        MotorConfigurationType type = dcMotor.getMotorType();
        Rotation orientation = type.getOrientation();

        double ticksPerRev = type.getTicksPerRev();

        Log.d("VV", "wrap32Motor(" + (dcMotor == null ? "null" : dcMotor) + ") " + orientation + ", " + ticksPerRev + " t/r");

        return new NinjaMotor(dcMotor, orientation,
                (int)type.getTicksPerRev() /* why does the SDK use double here, but motor interface accepts int ? */);
    }

    /**
     * Creates an ExtendedDcMotor from the given FTC SDK Motor with Neverest 20 behavior
     */
    public static ExtendedDcMotor asNeverest20(DcMotor dcMotor) {
        Log.d("VV", "asNeverest20(" + (dcMotor == null ? "null" : dcMotor) + ")");

        return new NinjaMotor(dcMotor, Rotation.CW, 560);
    }

    public static ExtendedDcMotor asNeverest20Orbital(DcMotor dcMotor) {
        return new NinjaMotor(dcMotor, Rotation.CCW, 537.6);  //nr 20 orbital)
    }

    public static ExtendedDcMotor asNeverest37Orbital(DcMotor dcMotor) {
        return new NinjaMotor(dcMotor, Rotation.CCW, 103);  //nr 3.7 orbital)
    }

    /**
     * Creates an ExtendedDcMotor from the given FTC SDK Motor with Neverest 40 behavior
     */
    public static ExtendedDcMotor asNeverest40(DcMotor dcMotor) {

        Log.d("VV", "asNeverest40(" + (dcMotor == null ? "null" : dcMotor) + ")");


        return new NinjaMotor(dcMotor, Rotation.CCW, 1120);
    }

    /**
     * Creates an ExtendedDcMotor from the given FTC SDK Motor with Neverest 60 behavior
     */
    public static ExtendedDcMotor asNeverest60(DcMotor dcMotor) {

        Log.d("VV", "asNeverest60(" + (dcMotor == null ? "null" : dcMotor) + ")");

        return new NinjaMotor(dcMotor, Rotation.CCW, 420);
    }

    /**
     * Creates an ExtendedDcMotor from the given FTC SDK Motor with Tetrix gear motor behavior
     */
    public static ExtendedDcMotor asTetrix(DcMotor dcMotor) {

        Log.d("VV", "asTetrix20(" + (dcMotor == null ? "null" : dcMotor) + ")");

        return new NinjaMotor(dcMotor, Rotation.CW, 1440);
    }

    /**
     * Private because we want to have code only use the factory methods
     */
    private NinjaMotor(DcMotor dcMotor, Rotation motorNativeDirection, double encoderCountsPerRevolution) {
        this.encoderCountsPerRevolution = encoderCountsPerRevolution;
        this.dcMotor = dcMotor;
        this.motorNativeDirection = motorNativeDirection;
        zeroEncoderValue = dcMotor.getCurrentPosition(); // it's expensive to reset this via the SDK!
        velocityTracker = new MotorVelocityTracker(dcMotor);
    }

    /**
     * Which way does the motor shaft rotate when plugged directly into power?
     */
    @Override
    public Rotation getMotorNativeDirection() {
        return motorNativeDirection;
    }

    /**
     * How many PPR does this motor produce?
     */
    @Override
    public double getEncoderCountsPerRevolution() {
        return encoderCountsPerRevolution;
    }

    /**
     * Sets the absolute target position for this motor to current position + relativePosition
     */
    @Override
    public int setRelativeTargetPosition(int relativePosition) {
        int currentMotorPosition = dcMotor.getCurrentPosition();
        int absolutePosition = currentMotorPosition + relativePosition;
        dcMotor.setTargetPosition(absolutePosition);
        Log.d("VV", "NinjaMotor sRTP() - r/c/a" + relativePosition + "/" + currentMotorPosition + "/" + absolutePosition);

        return absolutePosition;
    }

    /**
     * Returns the current relative position from the last time setRelativeTargetPosition() was
     * called. If setRelativeTargetPosition() has never been called, this will return the same
     * value as calling getCurrentPosition().
     */
    @Override
    public int getCurrentRelativePosition() {
        return dcMotor.getCurrentPosition() - absoluteTargetPosition;
    }

    @Override
    public Manufacturer getManufacturer() {
        return dcMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return dcMotor.getDeviceName();
    }

    @Override
    public int getTargetPosition() {
        return dcMotor.getTargetPosition();
    }

    @Override
    public void setPowerFloat() {
        dcMotor.setPowerFloat();
    }

    @Override
    public void setPower(double power) {
        dcMotor.setPower(power);
    }

    @Override
    public void close() {
        dcMotor.close();
    }

    @Override
    public boolean getPowerFloat() {
        return dcMotor.getPowerFloat();
    }

    @Override
    public void resetLogicalEncoderCount() {
        zeroEncoderValue = dcMotor.getCurrentPosition();
    }

    //@Override
    //public int getVelocity() {
    //    return velocityTracker.getVelocity();
    //}

    @Override
    public int getCurrentPosition() {

        int currentEncoderPosition = dcMotor.getCurrentPosition();

        if (currentEncoderPosition == 0 && Math.abs(prevEncoderPosition) > 1000) {
            // ESD event or encoder bug? Don't handle for now, just log
            // REF https://ftcforum.usfirst.org/forum/ftc-technology/58524-rev-expansion-hub-firmware-1-7-bug
            // REF https://www.reddit.com/r/FTC/comments/7hdbd4/rev_expansion_hub_flash_17_possible_issue/

            Log.e(Constants.LOG_TAG,
                    String.format("Detected possible ESD or encoder failure on port %d",
                            dcMotor.getPortNumber()));
        } else {
            prevEncoderPosition = currentEncoderPosition;
        }

        return currentEncoderPosition - zeroEncoderValue;
    }

    @Override
    public void setMode(RunMode mode) {
        dcMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return dcMotor.getMode();
    }

    @Override
    public DcMotorController getController() {
        return dcMotor.getController();
    }

    @Override
    public boolean isBusy() {
        return dcMotor.isBusy();
    }

    @Override
    public void setDirection(DcMotor.Direction direction) {
        dcMotor.setDirection(direction);
    }

    @Override
    public DcMotor.Direction getDirection() {
        return dcMotor.getDirection();
    }

    @Override
    public String getConnectionInfo() {
        return dcMotor.getConnectionInfo();
    }

    @Override
    public void setTargetPosition(int position) {
        dcMotor.setTargetPosition(position);
    }

    @Override
    public int getPortNumber() {
        return dcMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return dcMotor.getZeroPowerBehavior();
    }

    @Override
    public int getVersion() {
        return dcMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        dcMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public double getPower() {
        return dcMotor.getPower();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return dcMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        dcMotor.setMotorType(motorType);
    }

    public static class MotorVelocityTracker {
        private final DcMotor motor;

        public MotorVelocityTracker(DcMotor motor) {
            this.motor = motor;
            lastTimeSinceVelocityCheck = System.currentTimeMillis();
            lastPositionSinceVelocityCheck = motor.getCurrentPosition();
        }

        private long lastTimeSinceVelocityCheck;

        private int lastPositionSinceVelocityCheck;

        public int getVelocity() {
            long currentTime = System.currentTimeMillis();
            int currentPosition = motor.getCurrentPosition();

            if(currentTime == lastTimeSinceVelocityCheck) {
                // FIXME: Explain why +1 on current time
                return (int) ((currentPosition- lastPositionSinceVelocityCheck) / ((currentTime+1) - lastTimeSinceVelocityCheck));
            }

            long velocity = (currentPosition - lastPositionSinceVelocityCheck) / (currentTime - lastTimeSinceVelocityCheck);

            return (int)velocity;
        }
    }
}

