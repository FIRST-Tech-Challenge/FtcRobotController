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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware {
    /* Public OpMode members. */
    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor lift = null;
    public DcMotor upperLift;

    public AnalogInput toggleSwitch = null;
    public AnalogInput bottomSwitch;
    public AnalogInput upperDownSwitch;
    public AnalogInput upperUpSwitch;
    public AnalogInput gripSwitch;

    public Servo gripper = null;
    public Servo wrist = null;
    public CRServo gripperWheel = null;

    public BNO055IMU imu = null;

    public RevColorSensorV3 leftColor = null;
    public RevColorSensorV3 rightColor = null;

    public DigitalChannel redLED = null;
    public DigitalChannel greenLED = null;

    //public RevBlinkinLedDriver illuminate = null;

    public OpenCvWebcam webcam;
    public OpenCvWebcam webcam2;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive = hwMap.get(DcMotor.class, "fl");
        backLeftDrive = hwMap.get(DcMotor.class, "bl");
        frontRightDrive = hwMap.get(DcMotor.class, "fr");
        backRightDrive = hwMap.get(DcMotor.class, "br");

        lift = hwMap.get(DcMotor.class, "lift");
        upperLift = hwMap.get(DcMotor.class, "topLift");

        toggleSwitch = hwMap.get(AnalogInput.class, "toggle");
        bottomSwitch = hwMap.get(AnalogInput.class, "bottom");
        upperUpSwitch = hwMap.get(AnalogInput.class, "topUp");
        upperDownSwitch = hwMap.get(AnalogInput.class, "topDown");

        gripSwitch = hwMap.get(AnalogInput.class, "gripSwitch");

        gripper = hwMap.get(Servo.class, "grip");
        wrist = hwMap.get(Servo.class, "wrist");
        gripperWheel = hwMap.get(CRServo.class, "gw");

        imu = hwMap.get(BNO055IMU.class, "imu");

        redLED = hwMap.get(DigitalChannel.class, "red");
        greenLED = hwMap.get(DigitalChannel.class, "green");

        //rightColor = hwMap.get(RevColorSensorV3.class, "rcolor");
        //leftColor = hwMap.get(RevColorSensorV3.class, "lcolor");
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "webcam"));
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "webcam2"), cameraMonitorViewId);

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        //  bottomColorSensor = hwMap.get(RevColorSensorV3.class, "bottomColor");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        upperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.

    }

    public void drive(float y_axis, float x_axis, float rotation, float speedLimit) {
        double Protate = rotation / 4;
        double stick_x = x_axis * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = y_axis * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2);
        double theta = 0;
        double Px = 0;
        double Py = 0;
        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));
        frontLeftDrive.setPower((Py - rotation) * speedLimit);
        backLeftDrive.setPower((Px + rotation) * speedLimit);
        backRightDrive.setPower((Py + rotation) * speedLimit);
        frontRightDrive.setPower((Px - rotation) * speedLimit);
    }

    public void omniDrive(float y_axis, float x_axis, float rotation, float speedLimit) {
//        double theta = (Math.atan2(y_axis, x_axis)) /**- (Math.PI / 2)**/;
//        double Px = Math.sqrt(Math.pow(x_axis, 2) + Math.pow(y_axis, 2)) * (Math.sin(theta + Math.PI / 4));
//        double Py = Math.sqrt(Math.pow(x_axis, 2) + Math.pow(y_axis, 2)) * (Math.sin(theta - Math.PI / 4));
//        double Pr = rotation/4;
//
//        frontDrive.setPower((Py - Pr) * speedLimit);
//        backDrive.setPower((Py - Pr) * speedLimit);
//        rightDrive.setPower((Px - Pr) * speedLimit);
//        rightDrive.setPower((Px - Pr) * speedLimit);
        double denominator = Math.max(Math.abs(y_axis) + Math.abs(x_axis) + Math.abs(rotation), 1);
        double leftPower = (((y_axis + x_axis) * ((Math.PI / 8))) + rotation) / denominator;
        double frontPower = (((y_axis - x_axis) * ((Math.PI / 8))) - rotation) / denominator;
        double backPower = (((y_axis - x_axis) * ((Math.PI / 8))) + rotation) / denominator;
        double rightPower = (((y_axis + x_axis) * ((Math.PI / 8))) - rotation) / denominator;
        frontLeftDrive.setPower((frontPower) * speedLimit);
        backLeftDrive.setPower((backPower) * speedLimit);
        frontRightDrive.setPower((rightPower) * speedLimit);
        backRightDrive.setPower((leftPower) * speedLimit);
    }
}

