package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class betterDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        boolean leftTriggerAtRest = true;
        boolean rightTriggerAtRest = true;
        boolean calculatePower = false;
        double adjustment = 0;
        boolean setMotorPower = false;
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
            double rx = 0;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            if (gamepad1.left_trigger == 0) {
                leftTriggerAtRest = true;
            } else if (gamepad1.left_trigger == 1) {
                leftTriggerAtRest = false;
            }
            if (gamepad1.right_trigger == 0) {
                rightTriggerAtRest = true;
            } else if (gamepad1.right_trigger == 1) {
                rightTriggerAtRest = false;
            }
            if (leftTriggerAtRest && rightTriggerAtRest) {
                adjustment = 2;
                setMotorPower = true;
            }
            if (!leftTriggerAtRest) {
                adjustment = 4;
                setMotorPower = true;
            }
            if (!rightTriggerAtRest) {
                adjustment = 0.5;
                setMotorPower = true;
            }
            if (!leftTriggerAtRest && !rightTriggerAtRest) {
                adjustment = 2;
            }
            if (gamepad1.right_stick_x == 1) {
                frontLeftMotor.setPower(1/adjustment);
                frontRightMotor.setPower(1/adjustment);
                backLeftMotor.setPower(-1/adjustment);
                backRightMotor.setPower(-1/adjustment);
            }
            if (gamepad1.right_stick_x == -1) {
                frontLeftMotor.setPower(-1/adjustment);
                frontRightMotor.setPower(-1/adjustment);
                backLeftMotor.setPower(1/adjustment);
                backRightMotor.setPower(1/adjustment);
            }
            if (setMotorPower) {
                frontLeftMotor.setPower(frontLeftPower / adjustment);
                frontRightMotor.setPower(frontRightPower / adjustment);
                backLeftMotor.setPower(backLeftPower / adjustment);
                backRightMotor.setPower(backRightPower / adjustment);
                setMotorPower = false;
            }
        }
    }
}