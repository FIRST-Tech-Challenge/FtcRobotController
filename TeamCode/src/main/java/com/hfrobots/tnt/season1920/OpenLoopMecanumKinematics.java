/**
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1920;

import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.qualcomm.robotcore.util.Range;

import lombok.AllArgsConstructor;

@AllArgsConstructor
public class OpenLoopMecanumKinematics {
    private final RoadRunnerMecanumDriveREV mecanumDrive;

    public void driveCartesian(double xPower,
                               double yPower,
                               double rotationPower,
                               boolean inverted,
                               double gyroAngle,
                               boolean useEncoders)
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

        if (useEncoders) {
            mecanumDrive.enableEncoders();
        } else {
            mecanumDrive.disableEncoders();
        }

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

    private final static double MAX_MOTOR_OUTPUT = 1.0;

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

    protected void normalizeAndSetMotorPower(WheelSpeeds wheelSpeeds) {
        wheelSpeeds.normalize();
        wheelSpeeds.clipToMaxOutput();

//        leftFront.setPower(v);
//        leftRear.setPower(v1);
//        rightRear.setPower(v2);
//        rightFront.setPower(v3);

        mecanumDrive.setMotorPowers(wheelSpeeds.leftFront,
                wheelSpeeds.leftRear,
                wheelSpeeds.rightRear,
                wheelSpeeds.rightFront);
    }
}
