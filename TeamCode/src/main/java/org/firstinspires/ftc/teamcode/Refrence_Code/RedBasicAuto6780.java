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

package org.firstinspires.ftc.teamcode.Refrence_Code;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Timer;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="RedBasicAuto6780", group="Iterative OpMode")

public class RedBasicAuto6780 extends OpMode
{
    // Declare OpMode members.
    private IMU imu = null;
    private boolean isFlywheelUpToSpeed = false;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontIntake = null;
    private DcMotor middleIntake = null;
    private DcMotorEx outake = null;
    private Servo hood = null;
    private ElapsedTime autoTimer = new ElapsedTime();



    boolean isOuttakeOn = false;
    boolean isOuttakePressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        imu = hardwareMap.get(IMU.class, "imu");
        hood = hardwareMap.get(Servo.class, "hood");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontIntake = hardwareMap.get(DcMotor.class, "frontintake");
        middleIntake = hardwareMap.get(DcMotor.class, "middle_intake");

        outake = (DcMotorEx) hardwareMap.get(DcMotor.class, "outake");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        middleIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);



        outake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        RevHubOrientationOnRobot revHubOrintation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(revHubOrintation));

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        imu.resetYaw();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        autoTimer.reset();
    }

    @Override
    public void loop() {
        if (autoTimer.seconds() < .52){
            MoveRobot(0,0.7,0);
        }
        else if (autoTimer.seconds() < 2.72) {
            MoveRobot(0,0,0);

            if (IsFlywheelUpToSpeed() == true){
                middleIntake.setPower(1);
                frontIntake.setPower(1);
            }
        }

        else if (autoTimer.seconds() < 4) {
            MoveRobot(-0.9,0,0);
            middleIntake.setPower(0);
            outake.setPower(0);
            frontIntake.setPower(0);
        }
        else {
            MoveRobot(0,0,0);
        }




    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void MoveRobot( double strafe, double forwards, double turn) {

        // double strafe = (xInput * Math.sin(-heading)) - (yInput * Math.cos(-heading));
        // double forwards = (xInput * Math.cos(-heading)) + (yInput * Math.sin(-heading));

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double turnSpeedFactor = 0.45; // Adjust this value between 0 (no turn) and 1 (full turn)
        double scaledRx = Math.signum(turn) * turn * turn; // smoother turning

        double denominator = Math.max(Math.abs(forwards) + Math.abs(strafe) + Math.abs(scaledRx), 1);


        frontLeftDrive.setPower((forwards + strafe + scaledRx) / denominator * .9);
        backLeftDrive.setPower((forwards - strafe + scaledRx) / denominator * .9);
        frontRightDrive.setPower((forwards - strafe - scaledRx) / denominator *.9);
        backRightDrive.setPower((forwards + strafe - scaledRx) / denominator * .9);
    }

    private boolean IsFlywheelUpToSpeed (){
        telemetry.addData("outakespeed", outake.getVelocity() );
        if (outake.getVelocity() > 1900) {
            outake.setPower(.6);

        }
        else if (outake.getVelocity() < 1600) {
            outake.setPower(.9);

        }
        else {
            outake.setPower(.8);

        }

        if (outake.getVelocity() > 1750 && outake.getVelocity() < 1900)
        {
            return true;
        }
        else {
            return false;
        }
    }


}