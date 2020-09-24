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

/**
 * Given a left and right drive train, treat it as a tank drive
 */
public class TankDrive {
    private final DriveTrain leftDriveTrain;

    private final DriveTrain rightDriveTrain;

    /**
     * Creates a TankDrive with the given drive trains (which already have their rotations directions
     * set)
     */
    public TankDrive(DriveTrain leftDriveTrain, DriveTrain rightDriveTrain) {
        // TODO - this class could figure out which directions the wheels need to rotate...
        this.leftDriveTrain = leftDriveTrain;
        this.rightDriveTrain = rightDriveTrain;
    }

    /**
     * Are any motors powering this drive busy with PID control?
     */
    public boolean isBusy() {
        return leftDriveTrain.isBusy() || rightDriveTrain.isBusy();
    }

    /**
     * Sets the power levels for the motors on both sides simultaneously
     */
    public void drivePower(double leftPower, double rightPower) {
        leftDriveTrain.setPower(leftPower);
        rightDriveTrain.setPower(rightPower);
    }

    /**
     * Sets the direction the drive will consider being "forward" (to make working with encoders
     * easier)
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        if (direction.equals(DcMotorSimple.Direction.FORWARD)) {
            leftDriveTrain.setForwardDirection();
            rightDriveTrain.setForwardDirection();
        } else if (direction.equals(DcMotorSimple.Direction.REVERSE)) {
            leftDriveTrain.setReverseDirection();
            rightDriveTrain.setReverseDirection();
        }
    }

    /**
     * What encoder counts are required (from both sides) to drive the given number of inches.
     */
    public SidedTargetPositions getTargetPositionsForInchesTravel(double linearInchesToDrive) {
        int leftTargetPosition = leftDriveTrain.getAbsolutePositionForInchesTravel(linearInchesToDrive);
        int rightTargetPosition = rightDriveTrain.getAbsolutePositionForInchesTravel(linearInchesToDrive);

        return new SidedTargetPositions(leftTargetPosition, rightTargetPosition);
    }

    public int getEncoderCountsForInchesTravel(double linearInchesToDrive) {
        return leftDriveTrain.getEncoderCountsForDriveInches(linearInchesToDrive);
    }

    /**
     * Sets the RunMode for motors powering both sides of this TankDrive
     * @param runMode
     */
    public void setRunMode(DcMotor.RunMode runMode) {
        leftDriveTrain.setRunMode(runMode);
        rightDriveTrain.setRunMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftDriveTrain.setZeroPowerBehavior(zeroPowerBehavior);
        rightDriveTrain.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Return the encoder counts for the left and right side drive trains simultaneously.
     */
    public SidedTargetPositions getCurrentPositions() {
        int leftPosition = leftDriveTrain.getCurrentPosition();
        int rightPosition = rightDriveTrain.getCurrentPosition();

        return new SidedTargetPositions(leftPosition, rightPosition);
    }

    /**
     * Encoder values for two sides at once
     */
    public static class SidedTargetPositions {
        private final long leftTargetPosition;

        private final long rightTargetPosition;

        public SidedTargetPositions(long leftTargetPosition, long rightTargetPosition) {
            this.leftTargetPosition = leftTargetPosition;
            this.rightTargetPosition = rightTargetPosition;
        }

        public long getLeftTargetPosition() {
            return leftTargetPosition;
        }

        public long getRightTargetPosition() {
            return rightTargetPosition;
        }
    }
}
