/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumRobotController;

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
    public static double moveSpeed = 0.5;
    public static double moveDistance = 50;
    public static double fastSpeed = 0.5;
    public static double slowSpeed = 0.2;
    public static double linearScalar = 1.06;
    public static double angularScalar = 0.9954681619; //1.014
    public static double positionMultiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "PHOTOSENSOR");
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-0.142, 0, 180);
        myOtos.setOffset(offset);

        myOtos.setAngularScalar(angularScalar);
        myOtos.setLinearScalar(linearScalar);
        // Possibly multiply moveDistance by 0.9803921569

        myOtos.calibrateImu();
        myOtos.resetTracking();

        runtime = new ElapsedTime();

        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
////        claw.setDirection(Servo.Direction.REVERSE);
////        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");
//
        IMU gyro = hardwareMap.get(IMU.class, "imu2");


//        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, myOtos, this);
//
//        telemetry.addData("Status", "Initialized");
//        positionMultiplier = (54 - (0.005 * fastSpeed)) / 50;

        // Wait for the start button to be pressed
        waitForStart();
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        backLeft.setPower(-fastSpeed);
        backRight.setPower(-fastSpeed);
        frontLeft.setPower(-fastSpeed);
        frontRight.setPower(-fastSpeed);
        while (opModeIsActive()) {
            if (moveDistance - pos.y * positionMultiplier < 0.1) {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                break;
            } else if (moveDistance - pos.y * positionMultiplier < 5) {
                backLeft.setPower(-slowSpeed);
                backRight.setPower(-slowSpeed);
                frontLeft.setPower(-slowSpeed);
                frontRight.setPower(-slowSpeed);
            }
            telemetry.addData("X Position", pos.x * positionMultiplier);
            telemetry.addData("Y Position", pos.y * positionMultiplier);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
            pos = myOtos.getPosition();
        }

        while (opModeIsActive()) {
            double dy = moveDistance - pos.y * positionMultiplier;
            backLeft.setPower(-dy * 0.025);
            backRight.setPower(-dy * 0.025);
            frontLeft.setPower(-dy * 0.025);
            frontRight.setPower(-dy * 0.025);
            telemetry.addData("X Position", pos.x * positionMultiplier);
            telemetry.addData("Y Position", pos.y * positionMultiplier);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
            pos = myOtos.getPosition();
        }
//        backLeft.setVelocity(fastSpeed);
//        backRight.setVelocity(fastSpeed);
//        frontLeft.setVelocity(fastSpeed);
//        frontRight.setVelocity(fastSpeed);
//        while (opModeIsActive()) {
//            if (pos.y < 0.1) {
//                backLeft.setVelocity(0);
//                backRight.setVelocity(0);
//                frontLeft.setVelocity(0);
//                frontRight.setVelocity(0);
//            } else if (pos.y < 5) {
//                backLeft.setVelocity(slowSpeed);
//                backRight.setVelocity(slowSpeed);
//                frontLeft.setVelocity(slowSpeed);
//                frontRight.setVelocity(slowSpeed);
//            }
//            telemetry.addData("X Position", pos.x);
//            telemetry.addData("Y Position", pos.y);
//            telemetry.addData("Heading", pos.h);
//            telemetry.update();
//            pos = myOtos.getPosition();
//        }
//        robotController.positionDrive(new SparkFunOTOS.Pose2D(0, 50, 0), moveSpeed);
//        robotController.sleep(sleepTime);
//        robotController.sendTelemetry(telemetry);
    }
}
