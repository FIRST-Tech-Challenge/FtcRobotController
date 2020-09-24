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

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * An implementation of ExtendedDcMotor that lets us treat two motors as one. There is an assumption
 * that both motors are of the exact same make and model.
 *
 * There are some shortcomings, because of PID controller interactions between two motors this
 * class does not support RUN_TO_POSITION or RUN_WITH_ENCODERS run modes (you must provide your
 * own external PID control code), and this class also disables braking on the 2nd motor (for the
 * same reason). When some value is retrieved (currentPosition for example) it is only from the
 * first motor.
 */
public class DualDcMotor implements ExtendedDcMotor {
    private final ExtendedDcMotor firstMotor;

    private final ExtendedDcMotor secondMotor;

    public static DualDcMotor dualNeverest20(HardwareMap hardwareMap, String firstMotorName, String secondMotorName) {
        ExtendedDcMotor firstMotor = NinjaMotor.asNeverest20(hardwareMap.dcMotor.get(firstMotorName));
        ExtendedDcMotor secondMotor = NinjaMotor.asNeverest20(hardwareMap.dcMotor.get(secondMotorName));

        return new DualDcMotor(firstMotor, secondMotor);
    }

    public DualDcMotor(ExtendedDcMotor primaryMotor, ExtendedDcMotor secondaryMotor) {
        Log.d("VV", "Primary Motor: " + (primaryMotor == null ? "null" : primaryMotor) + " secondary motor: " + (secondaryMotor == null ? "null" : secondaryMotor));
        this.firstMotor = primaryMotor;
        this.secondMotor = secondaryMotor;
        // By default, we setup the second motor to just track first motor power/direction, not brake
        this.secondMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    }


    @Override
    public MotorConfigurationType getMotorType() {
        return firstMotor.getMotorType();
    }


    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        firstMotor.setMotorType(motorType);
        secondMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return firstMotor.getController(); // what do we do here, do we force/assume they use the same controller?
    }

    @Override
    public int getPortNumber() {
        return 0; // what do we do here?
    }

    /**
     * Same as DcMotor except that if setting braking mode, this class
     * will set the second motor to float to avoid PID control interactions
     */
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

        firstMotor.setZeroPowerBehavior(zeroPowerBehavior);

        // If we're using braking, we don't want the PIDs to fight
        // each other so set second motor to float
        if (ZeroPowerBehavior.BRAKE.equals(zeroPowerBehavior)) {
            secondMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        } else {
            secondMotor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {

        return firstMotor.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        firstMotor.setPowerFloat();
        secondMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return firstMotor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        throw new IllegalArgumentException("Not supported");
    }

    @Override
    public int getTargetPosition() {
        throw new IllegalArgumentException("Not supported");
    }

    @Override
    public boolean isBusy() {
        throw new IllegalArgumentException("Not supported");
    }

    @Override
    public int getCurrentPosition() {
        return firstMotor.getCurrentPosition() ; // since we return the first motor's absolute
                                                 // target on .setRelative(), we return the first
                                                 // here as well
    }

    @Override
    public void setMode(RunMode mode) {
        if (RunMode.RUN_USING_ENCODER.equals(mode)) {
            throw new IllegalArgumentException("Dual motors cannot use RUN_WITH_ENCODERS");
        }

        if (RunMode.RUN_TO_POSITION.equals(mode)) {
            throw new IllegalArgumentException("Dual motors cannot use RUN_TO_POSITION");
        }

        firstMotor.setMode(mode);
        secondMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return firstMotor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        firstMotor.setDirection(direction);
        secondMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        if (!firstMotor.getDirection().equals(secondMotor.getDirection())) {
            throw new RuntimeException("Directions are different between paired motors!");
        }

        return firstMotor.getDirection();
    }

    @Override
    public void setPower(double power) {
        firstMotor.setPower(power);
        secondMotor.setPower(power);
    }

    @Override
    public double getPower() {
        if (firstMotor.getPower() != secondMotor.getPower()) {
            throw new RuntimeException("Power is different between paired motors!");
        }

        return firstMotor.getPower();

    }

    @Override
    public Manufacturer getManufacturer() {
        return firstMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return firstMotor.getDeviceName(); // what do we do here?
    }

    @Override
    public String getConnectionInfo() {
        return firstMotor.getConnectionInfo(); // what do we do here?
    }

    @Override
    public int getVersion() {
        return firstMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        firstMotor.resetDeviceConfigurationForOpMode();
        secondMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        firstMotor.close();
        secondMotor.close();
    }


    @Override
    public Rotation getMotorNativeDirection() {
        Log.d("VV", "First motor: " + (firstMotor == null ? null : firstMotor));
        if (firstMotor != null) {
            Log.d("VV", "First motor native direction: " + firstMotor.getMotorNativeDirection());
        }

        return firstMotor.getMotorNativeDirection();
    }

    @Override
    public double getEncoderCountsPerRevolution() {
        return firstMotor.getEncoderCountsPerRevolution();
    }

    @Override
    public int setRelativeTargetPosition(int position) {
        int targetPosition = firstMotor.setRelativeTargetPosition(position);

        return targetPosition;
    }

    @Override
    public int getCurrentRelativePosition() {
        return firstMotor.getCurrentRelativePosition();
    }

    @Override
    public void resetLogicalEncoderCount() {
        firstMotor.resetLogicalEncoderCount();
        secondMotor.resetLogicalEncoderCount();
    }

    //@Override
    //public int getVelocity() {
    //    return 0;
    //}
}
