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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes four motors (left_front, right_front, left_back, right_back)
 * Sensors three (2M distance x2, color)
 *
 * This one file/class can be used by ALL of your OpModes.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

@Disabled
public class vvHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightRear;
    public DcMotor leftRear;
    public DcMotor leftArm;
    public DcMotor rightArm;
    public CRServo rightWheel;
    public CRServo leftWheel;
    public Servo drone;

    public ColorSensor colorSensor;
    public DistanceSensor distFront;
    public DistanceSensor distRear;


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public vvHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "FLM");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "FRM");
        rightRear = myOpMode.hardwareMap.get(DcMotor.class, "RRM");
        leftRear = myOpMode.hardwareMap.get(DcMotor.class, "RLM");

        // Define Servos
        rightWheel = myOpMode.hardwareMap.crservo.get("RSW");
        leftWheel = myOpMode.hardwareMap.crservo.get("LSW");

        //define and initialize sensors
        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "CLR");

        distFront = myOpMode.hardwareMap.get(DistanceSensor.class, "FDS");
        distRear = myOpMode.hardwareMap.get(DistanceSensor.class, "RDS");

        //Set the motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double Drive, double Turn) {
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        double RWPower = 0;
        double LWPower = 0;
        double drivePower = 0.5; //global drive power level

        double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (driveY + strafe + turn) / denominator;
        double backLeftPower = (driveY - strafe + turn) / denominator;
        double frontRightPower = (driveY - strafe - turn) / denominator;
        double backRightPower = (driveY + strafe - turn) / denominator;
    }
    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     */
    public void setDrivePower(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower, double drivePower) {
        // Output the values to the motor drives.
        leftFront.setPower(drivePower * frontLeftPower);
        leftRear.setPower(drivePower * backLeftPower);
        rightFront.setPower(drivePower * frontRightPower);
        rightRear.setPower(drivePower * backRightPower);
    }
    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param armPower driving power (-1.0 to 1.0)
     */
    public void setArmPower(double armPower) {
        leftArm.setPower(armPower);
        rightArm.setPower(armPower);
    }

    /**
     * Set the pickup servo powers
     *
     * @param LWPower
     * @param RWPower
     */
    public void setPickupPower(double LWPower, double RWPower) {
        leftWheel.setPower(LWPower);
        rightWheel.setPower(RWPower);
    }

    /**
     * Set the drone servo
     *
     * @param droneSet
     */
    public void setDronePosition(double droneSet) {
        drone.setPosition(droneSet);
    }
}
