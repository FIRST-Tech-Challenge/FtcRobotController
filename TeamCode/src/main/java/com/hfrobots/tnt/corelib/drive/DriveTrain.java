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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Represents a drive train that hides the complexity of gear ratios and changes in direction
 * to allow callers to essentially treat the output shaft as if it is the motor shaft.
 */
public class DriveTrain {

    private final Wheel wheel;
    private final ExtendedDcMotor driveMotor;
    private final double gearRatio;
    private final int direction;
    private final Rotation wheelRotatingForwardDirection;
    private final String name;
    private final DcMotorSimple.Direction forwardDirection;

    /**
     * Creates a drive train for the given wheel and gear train.
     *
     * @param wheel The wheel used in the drive train
     * @param wheelRotatingForwardDirection The direction the wheel rotates for forward motion
     * @param motor The ExtendedDcMotor used to power the drive train
     * @param gearTrain Array of Gear in the train in the order of driveMotor, ..., wheel
     */
    public DriveTrain(String name, Wheel wheel, Rotation wheelRotatingForwardDirection, ExtendedDcMotor motor, Gear[] gearTrain){
        this.name = name;
        Log.d("VV", this.name + " driveMotor is " + motor);


        this.wheel = wheel;
        this.wheelRotatingForwardDirection = wheelRotatingForwardDirection;
        this.driveMotor = motor;

        if (gearTrain == null) {
            throw new IllegalArgumentException("gear train needed");
        }
        if (gearTrain.length<2){
            throw new IllegalArgumentException("not enough gears");
        }
        Gear motorGear = gearTrain[0];
        Gear wheelGear = gearTrain[gearTrain.length - 1];

        gearRatio = wheelGear.getNumTeeth() / motorGear.getNumTeeth();

        // adjust for driveMotor vs. forward direction mismatch
        final boolean invertRotationalDirection;

        if (!driveMotor.getMotorNativeDirection().equals(wheelRotatingForwardDirection)) {
            invertRotationalDirection = true;
        } else {
            invertRotationalDirection = false;
        }

        // Even number of gears negates inverted rotational direction
        if ( gearTrain.length % 2 == 0 ) {
            direction = invertRotationalDirection ? 1 : -1;
        } else{
            direction = invertRotationalDirection ? -1 : 1;
        }

        Log.d("VV", this.name + " invertRotate=" + invertRotationalDirection + ", direction=" + direction);

        if (direction == -1) {
            Log.d("VV", this.name + " setting motor " + motor + " to REVERSE");

            // Do this instead of inverting power ... it handles encoders for us
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        this.forwardDirection = motor.getDirection(); // save for later
    }

    public DriveTrain(String name, Wheel wheel, Rotation wheelRotatingForwardDirection,
                      ExtendedDcMotor motor, Sprocket[] sprocketTrain){
        this.name = name;
        Log.d("VV", this.name + " driveMotor is " + motor);


        this.wheel = wheel;
        this.wheelRotatingForwardDirection = wheelRotatingForwardDirection;
        this.driveMotor = motor;

        if (sprocketTrain == null) {
            throw new IllegalArgumentException("sprocket train needed");
        }

        if (sprocketTrain.length<2){
            throw new IllegalArgumentException("not enough sprockets");
        }

        if (sprocketTrain.length > 2) {
            throw new IllegalArgumentException("too many sprockets");
        }

        Sprocket motorSprocket = sprocketTrain[0];
        Sprocket wheelSprocket = sprocketTrain[sprocketTrain.length - 1];

        gearRatio = wheelSprocket.getNumTeeth() / motorSprocket.getNumTeeth();

        // adjust for driveMotor vs. forward direction mismatch

        if (!driveMotor.getMotorNativeDirection().equals(wheelRotatingForwardDirection)) {
            direction = -1;
        } else {
            direction = 1;
        }

        Log.d("VV", this.name + ", direction=" + direction);

        if (direction == -1) {
            Log.d("VV", this.name + " setting motor " + motor + " to REVERSE");

            // Do this instead of inverting power ... it handles encoders for us
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        this.forwardDirection = motor.getDirection(); // save for later
    }

    public void setReverseDirection() {
        if (forwardDirection == DcMotorSimple.Direction.FORWARD) {
            Log.d("VV", "DriveTrain.setReverseDirection() - direction now REVERSE (forward is) " + forwardDirection);
            driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            Log.d("VV", "DriveTrain.setReverseDirection() - direction now FORWARD (forward is) " + forwardDirection);
            driveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void setForwardDirection() {
        driveMotor.setDirection(forwardDirection);
    }

    /**
     * Returns the current encoder count (position) of the motor that powers the drive train
     */
    public int getCurrentPosition() {
        return driveMotor.getCurrentPosition();
    }

    /**
     * Sets the power for the motor that powers the drive train
     */
    public void setPower(double power) {
        driveMotor.setPower(power);
    }

    /**
     * Returns the encoder count required to move the given number of inches (needed for external
     * P(ID) control.
     */
    public int getAbsolutePositionForInchesTravel(double linearInchesToDrive) {
        int requiredEncoderCounts = getEncoderCountsForDriveInches(linearInchesToDrive);

        int absoluteTargetPosition = driveMotor.getCurrentPosition() + requiredEncoderCounts;

        Log.d("VV", name + " drive train - requiredEncoder = " + requiredEncoderCounts
                + ", currentPosition = " + driveMotor.getCurrentPosition()
                + ", absoluteTargetPosition = " + absoluteTargetPosition);

        return absoluteTargetPosition;
    }

    /**
     * Drives the number of inches using RUN_TO_POSITION, returns the encoder count that represents
     * the distance. Does not work with DualDcMotors.
     */
    public int driveInches(double linearInchesToDrive, double power) {
        if (driveMotor instanceof DualDcMotor) {
            throw new IllegalArgumentException("DualDcMotor does not support built-in PID control - does not work with driveInches");
        }

        int requiredEncoderCounts = getEncoderCountsForDriveInches(linearInchesToDrive);
        int absoluteTargetPosition = driveMotor.setRelativeTargetPosition(requiredEncoderCounts);
        Log.d("VV", this + " driveInches, target=" + absoluteTargetPosition);
        driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveMotor.setPower(power);

        return absoluteTargetPosition;
    }

    /**
     * Sets the RunMode for the motor powering this drive train.
     */
    public void setRunMode(DcMotor.RunMode runMode) {
        driveMotor.setMode(runMode);
    }

    /**
     * Is the motor powering this drive train busy operating under PID control?
     */
    public boolean isBusy() {
        return driveMotor.isBusy();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        driveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public int getEncoderCountsForDriveInches(double linearInchesToDrive) {
        double wheelRevolutions = linearInchesToDrive / wheel.circumference;
        double motorRevolutions = wheelRevolutions * gearRatio;
        // this cast is safe -- we will never have an encoder count that will reach beyond the max of an int
        int encoderCounts = (int)(Math.round(driveMotor.getEncoderCountsPerRevolution() * motorRevolutions));

        return encoderCounts;
    }
}
