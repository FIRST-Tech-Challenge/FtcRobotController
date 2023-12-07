package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "moo")
public class moo extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("right stick", gamepad1.left_stick_y);
            telemetry.update();


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
        }
    }
}