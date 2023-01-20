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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
public class NewHardwareMap
{
    // position of servos

    /*final double ScoopL_Down = 0.88; //0.86
    final double ScoopLA_Down = ScoopL_Down - 0.06;
    final double ScoopL_PS = 0.55; //less is more up .53 PowerShots
    final double ScoopL_Up = 0.55; //less is more up .53
    final double ScoopL_WG = 0.42; //less is more up .40

    final double ScoopR_Down = 0.16; //0.17
    final double ScoopRA_Down = ScoopR_Down + 0.06;
    final double ScoopR_PS = 0.48;  //more is more up .47 PowerShots
    final double ScoopR_Up = 0.44;  //more is more up .47
    final double ScoopR_WG = 0.58;  //more is more up .60

    final double Hammer_stowed = 0.05; //0.05
    final double Hammer_fire = 0.35; //more is more out 0.35
    final double WG_locked  = 0.7;
    final double WG_locked_Auto  = 0.75;
    final double WG_un_lock = 0.1;
    final double WG_push = 0.3;

    final double FarShootVel = 2200;  //~2500max
    final double HighShootVel = 2100;  //~2500max * 0.81 = 2100
    final double PSshootVel = 1975;     //~2500max * 0.7 = 1950
    final double PSshootVelJ = 2175;*/


    final double CameraOne = 0.85;
    final double CameraTwo = 0.75;
    final double CameraThree = 0.9;

    //New Variables for Freight Frenzy
    int LiftNumber = 0;
    boolean LiftStop = true;
    int LiftLow = 1000;
    int LiftMed = 2000;
    int LiftHigh = 3000;

    public DcMotor FmotorRight = null;
    public DcMotor FmotorLeft = null;
    public DcMotor BmotorRight = null;
    public DcMotor BmotorLeft = null;
    //DcMotorEx ehShooter = null;
    public DcMotor LiftMotor = null;
    DcMotor Intake1 = null;
    DcMotor Intake2 = null;
    public DcMotor DuckMotor = null;
    //DcMotorSimple Shooter = null;    //SparkMini
    /*Servo ScoopL;
    Servo ScoopR;
    Servo Hammer;
    Servo WG_lock;*/
    Servo CameraPan;
    public CRServo Gray;
    public CRServo Green;
    public Servo FreightArm;
    //NormalizedColorSensor colorSensor;
    //DigitalChannel digitalTouch;
    public TouchSensor touch;
    public TouchSensor touch3;
    public DistanceSensor sensorRange;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public NewHardwareMap(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and created the configuration file.
         */

        //MOTORS
        FmotorRight = hwMap.dcMotor.get("right_motor"); //right1Motor
        FmotorRight.setDirection(DcMotor.Direction.REVERSE);
        FmotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BmotorRight = hwMap.dcMotor.get("b_right_motor"); //right2Motor
        BmotorRight.setDirection(DcMotor.Direction.REVERSE);
        BmotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FmotorLeft = hwMap.dcMotor.get("left_motor"); //left1Motor
        //FmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        FmotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BmotorLeft = hwMap.dcMotor.get("b_left_motor"); //left2Motor
        //BmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        BmotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftMotor = hwMap.dcMotor.get("lift_motor");
        //LiftMotor.setDirection(DcMotor.Direction.REVERSE);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake1 = hwMap.dcMotor.get("intake_left");
        Intake1.setDirection(DcMotor.Direction.REVERSE);
        Intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake2 = hwMap.dcMotor.get("intake_right");
        //Intake2.setDirection(DcMotor.Direction.REVERSE);
        Intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DuckMotor = hwMap.dcMotor.get("duck_motor"); //left2Motor
        //DuckMotor.setDirection(DcMotor.Direction.REVERSE);
        DuckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to our digitalTouch object.
        //digitalTouch = hwMap.get(DigitalChannel.class, "lift_limit");
        touch = hwMap.get(TouchSensor.class, "lift_limit");
        touch3 = hwMap.get(TouchSensor.class, "lift_limit_up");
        // set the digital channel to input.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
         /*public DistanceSensor getSensor() {
             return sensorRange;
         }*/

        //DcMotorEx ehShooter = hardwareMap.get(DcMotorEx.class, "eh_smotor");
        //ehShooter = hwMap.get(DcMotorEx.class,"eh_smotor");
        //ehShooter.setDirection(DcMotor.Direction.REVERSE);
        //ehShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ehShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //SparkMini
        //Shooter = hwMap.get(DcMotorSimple.class, "smotor"); //SparkMini

        //SERVOS
        /*ScoopL = hwMap.servo.get("scoop_l");
        ScoopR = hwMap.servo.get("scoop_r");
        Hammer = hwMap.servo.get("trigger");
        WG_lock = hwMap.servo.get("wg_lock");*/
        CameraPan = hwMap.servo.get("camera_servo");
        Gray = hwMap.crservo.get("gray");
        Green = hwMap.crservo.get("green");
        FreightArm = hwMap.servo.get("freight_arm");

        //Color Sensor
        //colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color");

    }
}