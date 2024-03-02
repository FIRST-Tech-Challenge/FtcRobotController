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

package org.firstinspires.ftc.teamcode.Testing.AdisCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="MovementFunctions", group="Linear Opmode")
//@Disabled
public class MovmentFunctionss extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor arm;
    Servo grip;


    double speed= 0.3;
    double ticks = 1680;
    double newLfTarget;
    private DcMotor lfDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor rbDrive = null;
    private DcMotor lbDrive = null;
    private DcMotor armBase = null;
    private DcMotor elbow = null;
    private Servo grabber = null;
    Servo servo;

    IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        lfDrive  = hardwareMap.get(DcMotor.class, "lf_drive");
        rfDrive = hardwareMap.get(DcMotor.class, "rf_drive");
        rbDrive  = hardwareMap.get(DcMotor.class, "rb_drive");
        lbDrive  = hardwareMap.get(DcMotor.class, "lb_drive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grip = hardwareMap.get(Servo.class, "grip");
        servo = hardwareMap.get(Servo.class, "drone");
        //FirstClaw  = hardwareMap.get(Servo.class, "GoodServo");
        //FirstClaw.setPosition(0);

//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection(DcMotor.Direction.FORWARD);
        rbDrive.setDirection(DcMotor.Direction.REVERSE);


        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry


        }
    }
    public void ForwardBlocks(long Blocks){
        lfDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        sleep(Blocks*600);
        lfDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rbDrive.setPower(0.0);
        rfDrive.setPower(0.0);
    }
    public void BackwardBlocks(long Blocks){
        lfDrive.setPower(-0.5);
        rfDrive.setPower(-0.5);
        rbDrive.setPower(-0.5);
        rfDrive.setPower(-0.5);
        sleep(Blocks*600);
        lfDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rbDrive.setPower(0.0);
        rfDrive.setPower(0.0);
    }
    public void LeftBlocks(long Blocks){
        lfDrive.setPower(0.5);
        rfDrive.setPower(-0.5);
        lbDrive.setPower(0.5);
        rfDrive.setPower(-0.5);
        sleep(Blocks*600);
        lfDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rbDrive.setPower(0.0);
        rfDrive.setPower(0.0);
    }
    public void RightBlocks(long Blocks){
        lfDrive.setPower(-0.5);
        rfDrive.setPower(0.5);
        lbDrive.setPower(-0.5);
        rfDrive.setPower(0.5);
        sleep(Blocks*600);
        lfDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rbDrive.setPower(0.0);
        rfDrive.setPower(0.0);
    }

    public void ForwardINches(long Inches){
        lfDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        sleep(Inches*600);
        lfDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rbDrive.setPower(0.0);
        rfDrive.setPower(0.0);
    }
    public void BackwardInches(long Inches){
        lfDrive.setPower(-0.5);
        rfDrive.setPower(-0.5);
        rbDrive.setPower(-0.5);
        rfDrive.setPower(-0.5);
        sleep(Inches*600);
        lfDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rbDrive.setPower(0.0);
        rfDrive.setPower(0.0);
    }
    public void LeftInches(long Inches){
        lfDrive.setPower(0.5);
        rfDrive.setPower(-0.5);
        lbDrive.setPower(0.5);
        rfDrive.setPower(-0.5);
        sleep(Inches*600);
        lfDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rbDrive.setPower(0.0);
        rfDrive.setPower(0.0);
    }
    public void RightInches(long Inches){
        lfDrive.setPower(-0.5);
        rfDrive.setPower(0.5);
        lbDrive.setPower(-0.5);
        rfDrive.setPower(0.5);
        sleep(Inches*600);
        lfDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rbDrive.setPower(0.0);
        rfDrive.setPower(0.0);
    }


    public void encoder(double turnage1){
        newLfTarget = ticks*turnage1;



        arm.setTargetPosition((int)newLfTarget);



        arm.setPower(speed);



        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        grip.setPosition(sp);

    }
}
