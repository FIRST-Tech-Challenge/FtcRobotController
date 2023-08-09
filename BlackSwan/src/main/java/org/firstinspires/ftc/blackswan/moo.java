package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "move x")
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
            if (gamepad1.left_stick_y > .7) {
                backLeft.setPower(1);
                backRight.setPower(1);
                frontLeft.setPower(1);
                frontRight.setPower(1);
            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            if (gamepad1.left_stick_y < -.7) {
                backLeft.setPower(-1);
                backRight.setPower(-1);
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            if (gamepad1.right_stick_x > .7) {
                backLeft.setPower(-1);
                backRight.setPower(1);
                frontLeft.setPower(-1);
                frontRight.setPower(1);

            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            if (gamepad1.right_stick_x < .7) {
                backLeft.setPower(1);
                backRight.setPower(-1);
                frontLeft.setPower(1);
                frontRight.setPower(-1);
            } else {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
        }
    }
}