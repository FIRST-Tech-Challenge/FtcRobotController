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

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */


public class Hardware
{
    /* Public OpMode members. */
    public Motor m0 = null;
    public Motor m1 = null;
    public Motor m2 = null;
    public Motor m3 = null;
    public Motor carousel = null;
    public Motor intake = null;
    public double MIN_ANGLE = 0;
    public double MAX_ANGLE = 180;
    MotorGroup frontMotors = new MotorGroup(m0, m1);
    MotorGroup backMotors = new MotorGroup(m2, m3);
    DifferentialDrive drive;
    MecanumDrive mecanum = new MecanumDrive(m1 , m1, m2, m3);
    DistanceSensor dist = null;



    /* local OpMode members. */
    HardwareMap hwMap   =  null;
    ServoEx servo = new SimpleServo(hwMap,"servo", MIN_ANGLE, MAX_ANGLE);

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        m0 = new Motor(hwMap, "m0");
        m1 = new Motor(hwMap, "m2");
        m2 = new Motor(hwMap, "m1");
        m3 = new Motor(hwMap, "m3");
        intake = new Motor(hwMap, "m4");
        carousel = new Motor(hwMap, "m5");
        dist = hwMap.get(DistanceSensor.class, "distsensor");

        m0.set(0);
        m1.set(0);
        m2.set(0);
        m3.set(0);
        intake.set(0);
        carousel.set(0);

        m0.setInverted(false);
        m1.setInverted(true);
        m2.setInverted(false);
        m3.setInverted(true);
        intake.setInverted(false);
        carousel.setInverted(false);

        // Set motors to run with/without encoders
        m0.setRunMode(Motor.RunMode.PositionControl);
        m1.setRunMode(Motor.RunMode.PositionControl);
        m2.setRunMode(Motor.RunMode.PositionControl);
        m3.setRunMode(Motor.RunMode.PositionControl);
        intake.setRunMode(Motor.RunMode.PositionControl);
        carousel.setRunMode(Motor.RunMode.PositionControl);

        m0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        servo.setRange(MIN_ANGLE, MAX_ANGLE);
        servo.setPosition(0);

        drive = new DifferentialDrive(frontMotors,backMotors);

    }
}