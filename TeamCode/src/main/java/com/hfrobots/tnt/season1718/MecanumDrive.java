/**
 Copyright (c) 2017 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1718;

import android.util.Log;

import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class MecanumDrive {
    protected ExtendedDcMotor leftFrontDriveMotor;

    protected ExtendedDcMotor rightFrontDriveMotor;

    protected ExtendedDcMotor leftRearDriveMotor;

    protected ExtendedDcMotor rightRearDriveMotor;

    protected ExtendedDcMotor[] motors;

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        protected ExtendedDcMotor leftFrontDriveMotor;

        protected ExtendedDcMotor rightFrontDriveMotor;

        protected ExtendedDcMotor leftRearDriveMotor;

        protected ExtendedDcMotor rightRearDriveMotor;

        public Builder leftFrontDriveMotor(ExtendedDcMotor motor) {
            leftFrontDriveMotor = motor;

            return this;
        }

        public Builder rightFrontDriveMotor(ExtendedDcMotor motor) {
            rightFrontDriveMotor = motor;

            return this;
        }

        public Builder leftRearDriveMotor(ExtendedDcMotor motor) {
            leftRearDriveMotor = motor;

            return this;
        }

        public Builder rightRearDriveMotor(ExtendedDcMotor motor) {
            rightRearDriveMotor = motor;

            return this;
        }

        MecanumDrive build() {
            if (leftFrontDriveMotor == null
                    || rightFrontDriveMotor == null
                    || leftRearDriveMotor == null
                    || rightRearDriveMotor == null) {
                throw new IllegalArgumentException("Error 404 four motors not found :/");
            }

            MecanumDrive built = new MecanumDrive();
            built.leftFrontDriveMotor = leftFrontDriveMotor;
            built.rightFrontDriveMotor = rightFrontDriveMotor;
            built.leftRearDriveMotor = leftRearDriveMotor;
            built.rightRearDriveMotor = rightRearDriveMotor;
            built.motors = new ExtendedDcMotor[] {leftFrontDriveMotor, rightFrontDriveMotor, leftRearDriveMotor, rightRearDriveMotor};

            return built;
        }

    }

    private MecanumDrive() {} // only use the builder!!!!

    protected void stopAllDriveMotors() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    protected static class WheelSpeeds {
        protected double leftFront;
        protected double rightFront;
        protected double leftRear;
        protected double rightRear;

        WheelSpeeds(double leftFront, double rightFront, double leftRear, double rightRear) {
            this.leftFront = leftFront;
            this.rightFront = rightFront;
            this.leftRear = leftRear;
            this.rightRear = rightRear;
        }

        void clipToMaxOutput() {
            leftFront = Range.clip(leftFront, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
            rightFront = Range.clip(rightFront, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
            leftRear = Range.clip(leftRear, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
            rightRear = Range.clip(rightRear, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
        }

        void normalize() {
            // (1) Find the maximum magnitude of all wheel power values regardless of sign

            // (2) If that value is more than MAX_MOTOR_OUTPUT, scale it so the maximum magnitude
            //     is MAX_MOTOR_OUTPUT and *all other values* are scaled relative to that

            double maxMagnitude = 0;

            if (Math.abs(leftFront) > 1.0) {
                maxMagnitude = Math.abs(leftFront);
            }

            if (Math.abs(rightFront) > 1.0) {
                maxMagnitude = Math.abs(rightFront);
            }

            if (Math.abs(leftRear) > 1.0) {
                maxMagnitude = Math.abs(leftRear);
            }

            if (Math.abs(rightRear) > 1.0) {
                maxMagnitude = Math.abs(rightRear);
            }

            if (maxMagnitude > 1.0) {
                leftFront /= maxMagnitude;
                rightFront /= maxMagnitude;
                leftRear /= maxMagnitude;
                rightRear /= maxMagnitude;
            }
        }

        public String toString() {
            return "Wheel speed: " + leftFront + ", " + rightFront + ", " + leftRear + ", " + rightRear;
        }
    }

    private final static double MAX_MOTOR_OUTPUT = 1.0;

    public void driveTank(double leftPower, double rightPower, boolean inverted) {
        leftPower = Range.clip(leftPower, -1.0, 1.0);
        rightPower = Range.clip(rightPower, -1.0, 1.0);

        if (leftFrontDriveMotor != null) {
            leftFrontDriveMotor.setPower(leftPower);
        }
        if (rightFrontDriveMotor != null) {
            rightFrontDriveMotor.setPower(rightPower);
        }
        if (leftRearDriveMotor != null) {
            leftRearDriveMotor.setPower(leftPower);
        }
        if (rightRearDriveMotor != null) {
            rightRearDriveMotor.setPower(rightPower);
        }
    }

    public void driveCartesian(double xPower, double yPower, double rotationPower, boolean inverted, double gyroAngle)
    {
        xPower = Range.clip(xPower, -1.0, 1.0);
        yPower = Range.clip(yPower, -1.0, 1.0);
        rotationPower = Range.clip(rotationPower, -1.0, 1.0);

        if (inverted) {
            xPower = -xPower;
            yPower = -yPower;
        }

        double cosA = Math.cos(Math.toRadians(gyroAngle));
        double sinA = Math.sin(Math.toRadians(gyroAngle));

        // Lauren - this formula should look familiar - do you recognize it?

        xPower = xPower * cosA - yPower * sinA;
        yPower = xPower * sinA + yPower * cosA;

        //if (gyroAssistEnabled)
        //{
        //    rotation += TrcUtil.clipRange(gyroAssistKp*(rotation - gyroRateScale*gyro.getZRotationRate().value));
        //}

        WheelSpeeds wheelSpeeds = new WheelSpeeds(
                xPower + yPower + rotationPower,  // left front
                -xPower + yPower - rotationPower, // right front
                -xPower + yPower + rotationPower, // left rear
                xPower + yPower - rotationPower); // right rear
        normalizeAndSetMotorPower(wheelSpeeds);
    }

    protected void normalizeAndSetMotorPower(WheelSpeeds wheelSpeeds) {
        wheelSpeeds.normalize();
        wheelSpeeds.clipToMaxOutput();

        if (leftFrontDriveMotor != null) {
            leftFrontDriveMotor.setPower(wheelSpeeds.leftFront);
        }
        if (rightFrontDriveMotor != null) {
            rightFrontDriveMotor.setPower(wheelSpeeds.rightFront);
        }
        if (leftRearDriveMotor != null) {
            leftRearDriveMotor.setPower(wheelSpeeds.leftRear);
        }
        if (rightRearDriveMotor != null) {
            rightRearDriveMotor.setPower(wheelSpeeds.rightRear);
        }
    }

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot will go in the given direction
     * and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void drivePolar(double magnitude, double direction, double rotation, boolean inverted)
    {
        magnitude = Range.clip((magnitude * Math.sqrt(2.0)), -1.0, 1.0);

        if (inverted) {
            direction += 180.0;
            direction %= 360.0;
        }

        double dirInRad = Math.toRadians(direction + 45.0 /* Why +45 degrees? */);
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        //if (gyroAssistEnabled)
        //{
        //    rotation += TrcUtil.clipRange(gyroAssistKp*(rotation - gyroRateScale*gyro.getZRotationRate().value));
        //}

        WheelSpeeds wheelSpeeds = new WheelSpeeds(sinD * magnitude + rotation,
                cosD * magnitude - rotation,
                cosD * magnitude + rotation,
                sinD * magnitude - rotation);
        normalizeAndSetMotorPower(wheelSpeeds);
    }

    public void resetOdometry() {
        for (ExtendedDcMotor motor : motors) {
            motor.resetLogicalEncoderCount();
        }
    }

    /**
     * Returns the encoder-valued position of the drive base
     * using the encoder values from all four motors
     */
    public int getYPosition() {
        //leftFrontDriveMotor.getCurrentPosition();
         return -((leftFrontDriveMotor.getCurrentPosition() + leftRearDriveMotor.getCurrentPosition() +
                rightFrontDriveMotor.getCurrentPosition() + rightRearDriveMotor.getCurrentPosition())/4);
        // // FIXME: 12/3/17; encoder move in opp direction than expected
    }

    public int getXPosition() {
        return -((leftFrontDriveMotor.getCurrentPosition() + rightRearDriveMotor.getCurrentPosition() -
                (leftRearDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition()))/4);
        // // FIXME: 12/3/17; encoders move in opp direction than expected
    }
}