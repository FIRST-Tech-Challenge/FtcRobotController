package org.firstinspires.ftc.Team19567.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class mecanumdrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor leftFrontLeftEnc = hardwareMap.dcMotor.get("leftFrontLeftEnc");
        DcMotor leftBackRightEnc = hardwareMap.dcMotor.get("leftBackRightEnc");
        DcMotor rightFrontBackEnc = hardwareMap.dcMotor.get("rightFrontBackEnc");
        DcMotor rightBackNoEnc = hardwareMap.dcMotor.get("rightBackNoEnc");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        leftFrontLeftEnc.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackRightEnc.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFrontLeftEnc.setPower(frontLeftPower);
            leftBackRightEnc.setPower(backLeftPower);
            rightFrontBackEnc.setPower(frontRightPower);
            rightBackNoEnc.setPower(backRightPower);
        }
    }
}