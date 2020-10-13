/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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
 */

package com.hfrobots.tnt.season2021;

import com.ftc9929.corelib.drive.OpenLoopMecanumKinematics;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import lombok.NonNull;

public class Drivebase extends OpenLoopMecanumKinematics {
    private final DcMotorEx rightFrontDriveMotor;
    private final DcMotorEx rightRearDriveMotor;
    private final DcMotorEx leftFrontDriveMotor;
    private final DcMotorEx leftRearDriveMotor;


    public Drivebase(@NonNull HardwareMap hardwareMap) {
        rightFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "rightFrontDriveMotor");
        rightRearDriveMotor = hardwareMap.get(DcMotorEx.class, "rightRearDriveMotor");
        leftFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "leftFrontDriveMotor");
        leftRearDriveMotor = hardwareMap.get(DcMotorEx.class, "leftRearDriveMotor");

        leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    protected void setMotorPowers(@NonNull WheelSpeeds wheelSpeeds) {
        rightFrontDriveMotor.setPower(wheelSpeeds.getRightFront());
        rightRearDriveMotor.setPower(wheelSpeeds.getRightRear());
        leftFrontDriveMotor.setPower(wheelSpeeds.getLeftFront());
        leftRearDriveMotor.setPower(wheelSpeeds.getLeftRear());
    }
}
