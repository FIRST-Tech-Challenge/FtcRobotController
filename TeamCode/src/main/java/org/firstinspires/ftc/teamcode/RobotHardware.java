/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/*
 * This file works in conjunction with the External Hardware Class sample called: RobotMovement.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample RobotMovement.java, and select TeleOp.
 *
 */

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)

    //ARM
    private DcMotorEx a1ArmMotor = null;
    private DcMotorEx a2ArmMotor = null;
    //private Servo claw = null;
    //private Servo leftWrist = null;
    //private Servo rightWrist = null;

    //DRIVE
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    //MISC
    private CRServo planeLauncher = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO =  0.5 ;
    public static final double WRIST_SERVO_SPEED =  0.02 ;  // sets rate to move servo
    public static double ARM_POWER  = 1;

    public static double a1extraPowerNeeded = 0;
    public static double a1powerApplied = 0;
    public static double a2extraPowerNeeded = 0;
    public static double a2powerApplied = 0;
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");
        a1ArmMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "a1");
        a2ArmMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "a2");
        planeLauncher = myOpMode.hardwareMap.get(CRServo.class, "plane_launcher");

        a1ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        a2ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        a1ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        a2ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        a1ArmMotor.setTargetPositionTolerance(0);
        a2ArmMotor.setTargetPositionTolerance(0);

        a1ArmMotor.setDirection(DcMotorEx.Direction.FORWARD);
        a2ArmMotor.setDirection(DcMotorEx.Direction.FORWARD);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        planeLauncher.setDirection(DcMotorSimple.Direction.REVERSE);


        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     */
    public void driveRobot(double left, double right) {

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(left, right);
    }
    public void setDrivePower(double leftWheel, double rightWheel) {
        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);
    }
    public void firePlaneLauncher(double planeLauncherStatus){
        planeLauncher.setPower(planeLauncherStatus);
    }

    public void setA2Power(double power)
    {
        if(power == 0){
            int holdAtTicks = a2ArmMotor.getCurrentPosition();
            a2ArmMotor.setPower(0.05);
            a2ArmMotor.setTargetPosition(holdAtTicks);
            a2ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            a2ArmMotor.setPower(power);
            a2ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void setA1Power(double power)
    {
        if(power == 0){
            int holdAtTicks = a1ArmMotor.getCurrentPosition();
            a1ArmMotor.setPower(0.05);
            a1ArmMotor.setTargetPosition(holdAtTicks);
            a1ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            a1ArmMotor.setPower(power);
            a1ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
//
//    public void setWristPositions(double offset) {
//        offset = Range.clip(offset, -0.5, 0.5);
//        leftWrist.setPosition(MID_SERVO + offset);
//        rightWrist.setPosition(MID_SERVO - offset);
//    }

    //TODO BIG FAT RISK, DO NOT TRY THIS BEFORE CHECKING SERVO
    public void setClaw(boolean clawState){
        //claw.setPosition(0.75);
        if(clawState){
        }
        else{
            //claw.setPosition((0.25));
        }
    }
}