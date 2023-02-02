package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Discotest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBack");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");
        DcMotor armMotor1 = hardwareMap.dcMotor.get("arm1");
        DcMotor armMotor2 = hardwareMap.dcMotor.get("arm2");

        Servo intake = hardwareMap.servo.get("intake");
        CRServo axon = hardwareMap.crservo.get("axon");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double armPower = gamepad2.left_stick_y;
            boolean dpad2Up = gamepad2.dpad_up;
            boolean dpad2Down = gamepad2.dpad_down;
            if (dpad2Up) {
                intake.setPosition(0.0);
            }
            if (dpad2Down) {
                intake.setPosition(1.0);
            }
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            armMotor1.setPower(armPower);
            armMotor2.setPower(-armPower);
        }
    }
}