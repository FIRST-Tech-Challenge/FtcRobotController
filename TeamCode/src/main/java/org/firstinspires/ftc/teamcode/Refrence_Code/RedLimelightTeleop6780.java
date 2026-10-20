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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name="RedLimelightTeleop6780", group="Iterative OpMode")

public class RedLimelightTeleop6780 extends OpMode
{
    // Declare OpMode members.
    private IMU imu = null;

    private PIDController pidController;
    private Timer timer;
    private boolean isFlywheelUpToSpeed = false;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontIntake = null;
    private DcMotor middleIntake = null;
    private DcMotorEx outake = null;
    private DcMotorEx outtake2 = null;
    private Servo ballStopper = null;
    private Servo hood = null;

    private Timer shootTimer;
    boolean isUpToSpeed;




    boolean isOuttakeOn = false;
    boolean isOuttakePressed = false;
    private Limelight3A limelight = null;

    boolean isAutoAlignPressed = false;
    boolean isAutoAlignOn = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        frontIntake = hardwareMap.get(DcMotor.class, "frontintake");
        middleIntake = hardwareMap.get(DcMotor.class, "middle_intake");

        outake = (DcMotorEx) hardwareMap.get(DcMotor.class, "outake");
        outtake2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "ottake2");
        ballStopper = hardwareMap.get(Servo.class, "1");
        hood = hardwareMap.get(Servo.class, "hood");
        shootTimer = new Timer();
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        middleIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);



        outake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RevHubOrientationOnRobot revHubOrintation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(revHubOrintation));

        pidController = new PIDController(1, 45, 1, 20, 0);
        // P= max power it will give the robot, Max error is when it will start slowing down,
        // KI will get over static friction,
        // KI active zone is when it will start looking for active zone
        pidController.SetLoop(true, -180,180);
        timer = new Timer();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

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
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        limelight.start(); // This tells Limelight to start looking!
    }

    @Override
    public void loop() {
        timer.Update();

        if (gamepad1.left_bumper){
            if (isAutoAlignPressed == false) {
                isAutoAlignOn = !isAutoAlignOn;
                isAutoAlignPressed = true;
            }
        }

        else {
            isAutoAlignPressed = false;
        }


        LLResult llResult = limelight.getLatestResult();




        //Drive Code
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx;

        if (isAutoAlignOn == false) {
            rx = gamepad1.right_stick_x;
        }
        else {
            rx = GetAAPower(llResult);
        }




        double turnSpeedFactor = 0.8; // Adjust this value between 0 (no turn) and 1 (full turn)
        double scaledRx = rx * turnSpeedFactor; // smoother turning

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(scaledRx), 1);


        frontLeftDrive.setPower((y + x + scaledRx) / denominator * .85);
        backLeftDrive.setPower((y - x + scaledRx) / denominator * .85);
        frontRightDrive.setPower((y - x - scaledRx) / denominator *.85);
        backRightDrive.setPower((y + x - scaledRx) / denominator * .85);


        // ---------Intake----------
        if (gamepad1.right_bumper)   {
            frontIntake.setPower(1);
            middleIntake.setPower(1);
        }
        else {
            frontIntake.setPower(0);
            middleIntake.setPower(0);
        }

        //------Reverse Intake-------
        if (gamepad1.y) {
            middleIntake.setPower(-1);
            frontIntake.setPower(-1);
        }
        else {
            middleIntake.setPower(0);
            frontIntake.setPower(0);
        }

        //-----------Outtake-----------
        if (gamepad1.right_trigger > 0.3){
            if (isOuttakePressed == false) {
                isOuttakeOn = !isOuttakeOn;
                isOuttakePressed = true;
            }
        }
        else {
            isOuttakePressed = false;
        }


        if (isOuttakeOn == true) {
            ballStopper.setPosition(0);

            //--------Mid shot---------
            if (gamepad2.left_trigger > .3) {
                hood.setPosition(1);
                if (isOuttakeOn == true) {

                    double velocity = outake.getVelocity();
                    telemetry.addData("Outake velocity", velocity);
                    // Spin up flywheels until target speed
                    if (velocity < 1400) {
                        outake.setVelocity(1550);
                        outtake2.setVelocity(1550);
                        middleIntake.setPower(0);
                        frontIntake.setPower(0);
                    }
                    else if (velocity > 1600) {
                        outake.setVelocity(1550);
                        outtake2.setVelocity(1550);
                        middleIntake.setPower(1);
                        frontIntake.setPower(1);
                    }
                }
            }
            //--------Close shot---------
            else if (gamepad2.right_trigger > .3) {
                hood.setPosition(1);
                if (isOuttakeOn == true) {

                    double velocity = outake.getVelocity();
                    telemetry.addData("Outake velocity", velocity);
                    hood.setPosition(1);
                    // Spin up flywheels until target speed
                    if (velocity < 1200) {
                        outake.setVelocity(1250);
                        outtake2.setVelocity(1250);
                        middleIntake.setPower(0);
                        frontIntake.setPower(0);
                    }
                    else if (velocity > 1300) {
                        outake.setVelocity(1250);
                        outtake2.setVelocity(1150);
                        middleIntake.setPower(1);
                        frontIntake.setPower(1);
                    }
                }
            }
            //-----------Far shot---------
            else {
                hood.setPosition(.9);
                double velocity = outake.getVelocity();
                telemetry.addData("Outake velocity", velocity);

                // Spin up flywheels until target speed
                if (velocity < 2200) {
                    outake.setPower(1);
                    outtake2.setPower(1);
                    middleIntake.setPower(0);
                    frontIntake.setPower(0);
                }
                else if (velocity > 2200) {
                    outake.setPower(1);
                    outtake2.setPower(1);
                    middleIntake.setPower(1);
                    frontIntake.setPower(1);
                }
            }
        }
        else {
            outake.setPower(0);
            outtake2.setPower(0);
            middleIntake.setPower(0);
            frontIntake.setPower(0);
            ballStopper.setPosition(.3);
        }




        /*if (gamepad1.a){
            outake.setPower(1);
            middleIntake.setPower(1);
            frontIntake.setPower(1);
        }
*/

        if (gamepad1.left_stick_button){
            frontIntake.setPower(0);
            middleIntake.setPower(0);
            outake.setPower(0);

        }


       /* if (gamepad1.a){
            ballStopper.setPosition(0.3);
        }
        if (gamepad1.b){
            ballStopper.setPosition(0);
        }*/

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private double GetAAPower(LLResult llResult) {
        double aprilTagRotation = llResult.getTx();// Basically this tell the robot hey you are X amount of degrees off from facing the goal.

        telemetry.addData("tx: ", aprilTagRotation);

        double targetHeading = -imu.getRobotYawPitchRollAngles().getYaw() - aprilTagRotation;// this is the actual thing that tells you hey this is how many degrees it needs to turn. (Math)

        telemetry.addData("targetHeading: ", targetHeading);

        double turnPower = pidController.Calculate(-imu.getRobotYawPitchRollAngles().getYaw(), targetHeading, timer.deltaTime) / 45;// this tells the robot how much power it needs to move it to the proper degrees

        telemetry.addData("turnPower: ", turnPower);

        return turnPower;
    }

    private boolean CanShoot (Position robotPosition) {

        telemetry.addData("x", robotPosition.x);
        telemetry.addData("y", robotPosition.y);
        telemetry.addData("z: ", robotPosition.z);

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