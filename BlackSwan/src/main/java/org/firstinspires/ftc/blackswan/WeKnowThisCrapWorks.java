package org.firstinspires.ftc.blackswan;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="WeKnowThisCrapWorks")

public class WeKnowThisCrapWorks extends LinearOpMode {

    double MAX_SPEED = 0.9;
    int LiftPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Defining different motor/servos and what they are called in the configuration file
        DcMotor frontLeft, backLeft, frontRight, backRight;

        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backRight = hardwareMap.get(DcMotor.class, "backright");

       frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Turns robot right
            if (gamepad1.right_stick_x > 0.1) {
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                // Turns robot left
            } else if (gamepad1.right_stick_x < -0.1) {
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
            } else if (gamepad1.left_stick_x < -0.25 && gamepad1.left_stick_y < -0.25) {
                // Moves the robot UpLeft
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(0);
                frontLeft.setPower(0);
            } else if (gamepad1.left_stick_x > 0.25 && gamepad1.left_stick_y < -0.25) {
                // Moves the robot UpRight
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(0);
                frontRight.setPower(0);
            } else if (gamepad1.left_stick_x > 0.25 && gamepad1.left_stick_y > 0.25) {
                // Moves the robot DownRight
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(0);
                frontLeft.setPower(0);
            } else if (gamepad1.left_stick_x < -0.25 && gamepad1.left_stick_y > 0.25) {
                // Moves the robot DownLeft
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(0);
                frontRight.setPower(0);
            } else if (gamepad1.left_stick_y > 0.1) {
                // Moves the robot Down
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
            } else if (gamepad1.left_stick_y < -0.1) {
                // Moves the robot Up
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
            } else if (gamepad1.left_stick_x < -0.1) {
                // Moves the robot Right
                frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
            } else if (gamepad1.left_stick_x > 0.1) {
                // Moves the robot Left
                frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
            } else {
                // Sets motor power to zero so it does not attack us
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
        }
    }
}
