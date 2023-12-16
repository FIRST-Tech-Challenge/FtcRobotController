package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "moo")
public class moo extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor slideLeft = hardwareMap.dcMotor.get("slideLeft");
        DcMotor slideRight = hardwareMap.dcMotor.get("slideRight");

        Servo clawUp = hardwareMap.servo.get("clawUp");
        Servo clawUp2 = hardwareMap.servo.get("clawUp2");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("right stick", gamepad1.left_stick_y);


            if (gamepad1.left_stick_y > .7) {
                backLeft.setPower(-gamepad1.left_stick_y);
                backRight.setPower(-gamepad1.left_stick_y);
                frontLeft.setPower(-gamepad1.left_stick_y);
                frontRight.setPower(-gamepad1.left_stick_y);
            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            if (gamepad1.left_stick_y < .7) {
                backLeft.setPower(-gamepad1.left_stick_y);
                backRight.setPower(-gamepad1.left_stick_y);
                frontLeft.setPower(-gamepad1.left_stick_y);
                frontRight.setPower(-gamepad1.left_stick_y);
            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            if (gamepad1.right_stick_x > -.7) {
                backLeft.setPower(gamepad1.right_stick_x);
                backRight.setPower(-gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x);
                frontRight.setPower(-gamepad1.right_stick_x);

            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            if (gamepad1.right_stick_x < .7) {
                backLeft.setPower(gamepad1.right_stick_x);
                backRight.setPower(-gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x);
                frontRight.setPower(-gamepad1.right_stick_x);
            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                if (gamepad1.left_stick_x < .7) {
                    backLeft.setPower(-gamepad1.left_stick_x);
                    backRight.setPower(gamepad1.left_stick_x);
                    frontLeft.setPower(gamepad1.left_stick_x);
                    frontRight.setPower(-gamepad1.left_stick_x);
                } else {
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                }
                if (gamepad1.left_stick_x > -.7) {
                    backLeft.setPower(gamepad1.left_stick_x);
                    backRight.setPower(-gamepad1.left_stick_x);
                    frontLeft.setPower(-gamepad1.left_stick_x);
                    frontRight.setPower(gamepad1.left_stick_x);
                } else {
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                }
            }

            telemetry.addData("left slide", slideLeft.getCurrentPosition());
            telemetry.addData("right slide", slideRight.getCurrentPosition());
            if (gamepad2.left_stick_y < -.7 && slideLeft.getCurrentPosition()<1000 && slideRight.getCurrentPosition()<1000) {
                slideLeft.setPower(.5);
                slideRight.setPower(.5);
            } else if (gamepad2.left_stick_y>0.7 && slideLeft.getCurrentPosition()>0 && slideRight.getCurrentPosition()>0){
                slideLeft.setPower(-0.5);
                slideRight.setPower(-0.5);
            } else {
                slideLeft.setPower(0);
                slideRight.setPower(0);
            }

            telemetry.update();
                }

            }
        }