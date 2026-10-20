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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.core.PIDController;
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

@Autonomous(name="LimelightAuto6780", group="Iterative OpMode")

public class LimelightAuto6780 extends OpMode
{
    // Declare OpMode members.
    private IMU imu = null;

    private PIDController pidController;
    private Timer timer;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor backLeftDrive = null;
    private Limelight3A limelight = null;
    private Servo lights = null;


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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        lights = hardwareMap.get(Servo.class, "lights");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);




        pidController = new PIDController(0.8, 45, .35, 15, 0);
        pidController.SetLoop(true, -180,180);
        timer = new Timer();

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
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        autoTimer.reset();
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        limelight.start(); // This tells Limelight to start looking!
    }

    @Override
    public void loop() {
        timer.Update();


        LLResult llResult = limelight.getLatestResult();
        double aprilTagRotation = llResult.getTx();

        telemetry.addData("Yaw: ", llResult.getBotpose().getOrientation().getYaw());
        telemetry.addData("pitch: ", llResult.getBotpose().getOrientation().getPitch());
        telemetry.addData("roll: ", llResult.getBotpose().getOrientation().getRoll());
        telemetry.addData("x: ", llResult.getTx());
        telemetry.addData("y: ", llResult.getTy());

        MoveRobot(0,0,-imu.getRobotYawPitchRollAngles().getYaw() - aprilTagRotation);

        if (CanShoot(llResult.getBotpose().getPosition())) {
            lights.setPosition(.500);
        }
        else {
            lights.setPosition(.277);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void MoveRobot( double strafe, double forwards, double targetHeading) {

        // double strafe = (xInput * Math.sin(-heading)) - (yInput * Math.cos(-heading));
        // double forwards = (xInput * Math.cos(-heading)) + (yInput * Math.sin(-heading));

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double turnPower = pidController.Calculate(-imu.getRobotYawPitchRollAngles().getYaw(), targetHeading, timer.deltaTime) / 45;

        double denominator = Math.max(Math.abs(forwards) + Math.abs(strafe) + Math.abs(turnPower), 1);



        frontLeftDrive.setPower((forwards + strafe + turnPower) / denominator * .9);
        backLeftDrive.setPower((forwards - strafe + turnPower) / denominator * .9);
        frontRightDrive.setPower((forwards - strafe - turnPower) / denominator *.9);
        backRightDrive.setPower((forwards + strafe - turnPower) / denominator * .9);

        telemetry.addData("currentHeading ", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("targetHeading ", targetHeading);
        telemetry.addData("currentPower ", turnPower);
        telemetry.addData("timer.deltaTime ", timer.deltaTime);
    }

    private boolean CanShoot (Position robotPosition) {

        double a = robotPosition.z;
        double b = robotPosition.x;
        double c = Math.sqrt((a * a) + (b * b));

        if (c > 1.5 && c < 2) {
            return true;
        }
        else {
            return false;
        }

    }


}