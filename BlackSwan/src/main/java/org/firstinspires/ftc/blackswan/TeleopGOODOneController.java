package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleopGOODOneController extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
        DcMotor linearslide = hardwareMap.dcMotor.get("linearSlide");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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

            if (gamepad2.dpad_up) {
                linearslide.setPower(1);
            } else if (gamepad2.dpad_down) {
                linearslide.setPower(-1);
            } else {
                linearslide.setPower(0);
            }

            if (gamepad2.dpad_right) { // Dumps cup right
                linearslide.setTargetPosition(500);
                linearslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearslide.setPower(.5);
            }

            if (gamepad2.dpad_left) { // Dumps cup right
                linearslide.setTargetPosition(500);
                linearslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearslide.setPower(.5);
            }

//            if (gamepad2.dpad_up) { // Dumps cup right
//                linearslide.setTargetPosition(500);
//                linearslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                linearslide.setPower(.5);
//            }

            telemetry.addData("GamepadX", x);
            telemetry.addData("GamepadY", y);
            telemetry.addData("Linear Slide Position", linearslide.getCurrentPosition());
            telemetry.update();

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
    }
}