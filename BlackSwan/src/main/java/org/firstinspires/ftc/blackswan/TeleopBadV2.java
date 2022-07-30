package org.firstinspires.ftc.blackswan;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleopBadV2")

public class TeleopBadV2 extends LinearOpMode {

    double MAX_SPEED = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        // Defining different motor/servos and what they are called in the configuration file
        DcMotor frontLeft, backLeft, frontRight, backRight, slide, carousel, intake;
        Servo dump;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        carousel = hardwareMap.get(DcMotor.class, "carousel");

        slide = hardwareMap.get(DcMotor.class, "slide");

        intake = hardwareMap.get(DcMotor.class, "intake");

        dump = hardwareMap.get(Servo.class, "dump");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        slide.setZeroPowerBehavior(BRAKE);

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

            // Dumps the cup which is attached to the linear slides
            if (gamepad2.dpad_left) { // Dumps cup left
                dump.setPosition(0.90);
                sleep(800);
                dump.setPosition(0.52);
            }
            if (gamepad2.dpad_right) { // Dumps cup right
                dump.setPosition(0.1);
                sleep(800);
                dump.setPosition(0.52);
            }

            // Spins the motor for the linear slides
            if (gamepad2.dpad_up) { // Moves Slides Up
                slide.setPower(1);
            } else if (gamepad2.dpad_down) { // Moves Slides Down
                slide.setPower(-0.5);
            } else {
                slide.setPower(0);
            }

            // Spins the intake
            if (gamepad2.right_trigger > 0.1) {
                intake.setPower(-1);
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            turnDuck(carousel);

        }
    }

    protected void turnDuck(DcMotor carousel){
        if(gamepad2.right_bumper){
            carousel.setPower(-0.5 );
        } else  if (gamepad2.left_bumper){
            carousel.setPower(0.5);
        }else {
            carousel.setPower(0);
        }

    }

}

// Hello human it seems you have scrolled to the end of the code i hope you have a great rest of your day :)