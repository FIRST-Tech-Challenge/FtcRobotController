/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for the robot.
 * <p>
 */
public class HuskyBot {
    /* Public OpMode members. */
    public DcMotorEx frontLeftDrive = null;
    public DcMotorEx frontRightDrive = null;
    public DcMotorEx rearLeftDrive = null;
    public DcMotorEx rearRightDrive = null;

    public DcMotor intaker = null;
    public DcMotorEx arm = null;
    public DcMotor claw = null;
    public DcMotor spinner = null;

    public TouchSensor armLimit = null;
    public DistanceSensor distanceSensorFrontLeft = null;
    public DistanceSensor distanceSensorFrontRight = null;
    public DistanceSensor distanceSensorBack = null;
    public DistanceSensor distanceSensorLeft = null;
    public DistanceSensor distanceSensorRight = null;

    // HD Hex Motor = 28 ticks per rev. See https://docs.revrobotics.com/rev-control-system/sensors/encoders
    // max free speed = 100 rps. See https://www.revrobotics.com/rev-41-1600/
    // S
    public static final double VELOCITY_CONSTANT = 28 * 100;

    public static final int ARM_LEVEL_0 = 0;
    public static final int ARM_LEVEL_1 = 280;
    public static final int ARM_LEVEL_2 = 460;
    public static final int ARM_LEVEL_3 = 640;

    public static final double INTAKER_POWER_IN = 0.8;
    public static final double INTAKER_POWER_OUT = -0.7;

    public static final int ARM_LOW_LIMIT = 0;
    public static final int ARM_HIGH_LIMIT = 750;

    public static final double SPINNER_POWER = 1;
    public static final double SPINNER_POWER_INCREMENT = 0.15;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    public HuskyBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive = hwMap.get(DcMotorEx.class, "front_left_drive");
        rearLeftDrive = hwMap.get(DcMotorEx.class, "rear_left_drive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "front_right_drive");
        rearRightDrive = hwMap.get(DcMotorEx.class, "rear_right_drive");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intaker = hwMap.get(DcMotor.class, "intaker");
        arm = hwMap.get(DcMotorEx.class, "arm");
//        claw = hwMap.get(DcMotor.class, "claw_motor");
        spinner = hwMap.get(DcMotor.class, "spinner");

        armLimit = hwMap.get(TouchSensor.class, "armLimit");
        distanceSensorFrontLeft = hwMap.get(DistanceSensor.class, "distanceFrontLeft");
        distanceSensorFrontRight = hwMap.get(DistanceSensor.class, "distanceFrontRight");
        distanceSensorBack = hwMap.get(DistanceSensor.class, "distanceBack");
        distanceSensorLeft = hwMap.get(DistanceSensor.class, "distanceLeft");
        distanceSensorRight = hwMap.get(DistanceSensor.class, "distanceRight");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        intaker.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
        intaker.setPower(0);
//        claw.setPower(0);
        spinner.setPower(0);

        // this base configuration sets the drive motors to run without encoders and the arm motor
        // to run with encoder. if any opmode requires different setting, that should be changed in
        // the opmode itself
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // https://docs.google.com/document/u/1/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic
        frontLeftDrive.setVelocityPIDFCoefficients(1.82, 0.182, 0, 18.2);
        frontLeftDrive.setPositionPIDFCoefficients(5.0);
        rearLeftDrive.setVelocityPIDFCoefficients(1.18, 0.118, 0, 11.8);
        rearLeftDrive.setPositionPIDFCoefficients(5.0);
        frontRightDrive.setVelocityPIDFCoefficients(1.43, 0.143, 0, 14.3);
        frontRightDrive.setPositionPIDFCoefficients(5.0);
        rearRightDrive.setVelocityPIDFCoefficients(1.27, 0.127, 0, 12.7);
        rearRightDrive.setPositionPIDFCoefficients(5.0);

        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double frontLeftPower, double rearLeftPower, double frontRightPower,
                       double rearRightPower) {
        frontLeftDrive.setPower(frontLeftPower);
        rearLeftDrive.setPower(rearLeftPower);
        frontRightDrive.setPower(frontRightPower);
        rearRightDrive.setPower(rearRightPower);
    }

    public void driveStop() {
        drive(0, 0, 0, 0);
    }

}

