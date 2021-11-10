//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MotorTest", group = "TeleOp")
public class TestMotorNewChassis extends LinearOpMode{
    // Declaring motor
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    Servo servoGrabber;

    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        // Initialize the motors and servos
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        servoGrabber = hardwareMap.servo.get("servoGrabber");

        // Set starting position of servos
        servoGrabber.setPosition(0.3);

        // Set direction of the motors
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // Set power of motors
        while (opModeIsActive()) {
            motorBackLeft.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            motorFrontLeft.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            motorBackRight.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
            motorFrontRight.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
        }

        // Grabber code
        if(gamepad1.x) {
            servoGrabber.setPosition(0);
        }
        else if(gamepad1.a) {
            servoGrabber.setPosition(1.0);
        }
    }
    public void pauseMillis(double time) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
            idle();
        }
    }
}