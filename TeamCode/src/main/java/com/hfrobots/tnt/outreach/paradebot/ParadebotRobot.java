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

package com.hfrobots.tnt.outreach.paradebot;

import android.util.Log;

import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

/**
 * This class knows about the configurations/placement of hardware on the robot, and contains
 * the code to make the robot perform actions (drive, move mechanisms, etc) in response to
 * inputs (human or autonomous).
 *
 * It does not know about the hardware map, tele-op controls, state machines, etc.
 */
public class ParadebotRobot {
    // Drivebase
    protected final ExtendedDcMotor leftDriveMotor;

    protected final ExtendedDcMotor rightDriveMotor;

    protected final Servo flagWaverServo;

    public ParadebotRobot(ExtendedDcMotor leftDriveMotor, ExtendedDcMotor rightDriveMotor, Servo flagWaverServo) {
        this.leftDriveMotor = leftDriveMotor;
        this.rightDriveMotor = rightDriveMotor;
        this.flagWaverServo = flagWaverServo;

        setupDrivebase();
    }

    protected void setupDrivebase() {
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (!leftDriveMotor.getMotorNativeDirection().equals(Rotation.CCW)) {
            leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (!rightDriveMotor.getMotorNativeDirection().equals(Rotation.CW)) {
            rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

    }

    public void drive(double leftPower, double rightPower) {
        leftDriveMotor.setPower(leftPower);

        rightDriveMotor.setPower(rightPower);
    }

    public void hardStop() {
        drive(0, 0);
    }

    public void setFlagPosition(double position) {
        flagWaverServo.setPosition(position);
    }
}
