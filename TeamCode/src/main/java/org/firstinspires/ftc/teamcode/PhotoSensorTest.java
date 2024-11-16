/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_otos".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.sparkfun.com/products/24904
 */
@Config
@TeleOp(name = "Photo Sensor Test", group = "TeleOp")
public class PhotoSensorTest extends LinearOpMode {
    // Create an instance of the sensor
    SparkFunOTOS myOtos;
    MecanumRobotController robotController;
    ElapsedTime runtime;
    public static double sleepTime = 40;
    public static double turnSpeed = 1.75;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "PHOTOSENSOR");
        runtime = new ElapsedTime();

        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
////        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
////        claw.setDirection(Servo.Direction.REVERSE);
////        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");
//
        IMU gyro = hardwareMap.get(IMU.class, "imu2");


        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, myOtos, this);
//
//        telemetry.addData("Status", "Initialized");

        // Wait for the start button to be pressed
        waitForStart();
        while (runtime.seconds() < 35) {
            robotController.continuousDrive(0, 0, turnSpeed);
        }
        robotController.turnTo(0, turnSpeed);
        robotController.sleep(sleepTime);
        robotController.sendTelemetry(telemetry);
    }
}
