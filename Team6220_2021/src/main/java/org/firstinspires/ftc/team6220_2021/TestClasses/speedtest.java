//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Speed Test", group = "TeleOp")
public class speedtest extends LinearOpMode{
    // Declaring motors and servos
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    double x = 0.7;

    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        //Initialize the motors and servos
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");

        //Set direction of the motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Declare variables
        int position = 0;
        boolean isPressed = false;
        double motorPower = 0.9;
        double increase = 1;
        double oldPosition = 0;

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //Set power of motors
        while (opModeIsActive()) {
            if (gamepad1.b) {
                motorFrontRight.setPower(gamepad1.left_stick_y);
            }
            else if (gamepad1.x) {
                motorFrontLeft.setPower(gamepad1.left_stick_y);
            }
            else if (gamepad1.a) {
                motorBackRight.setPower(gamepad1.left_stick_y);
            }
            else if (gamepad1.y) {
                motorBackLeft.setPower(gamepad1.left_stick_y);
            }
        }
    }
    public void pauseMillis(double time) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
            idle();
        }
    }
}