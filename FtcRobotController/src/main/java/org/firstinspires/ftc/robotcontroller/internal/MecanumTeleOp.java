package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Controller Mecanum Drive")
public class MecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor sliderMotor = hardwareMap.dcMotor.get("sliderMotor");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        Servo dropperServo = hardwareMap.servo.get("dropperServo");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1; // Counter imperfect strafing
            double rx = -gamepad1.right_stick_x;
            double sliderUp = gamepad1.right_trigger;
            double sliderDown = -gamepad1.left_trigger;
            boolean dropperDown = gamepad1.dpad_up;
            boolean dropperUp = gamepad1.dpad_down;
            boolean armUp = gamepad1.a;
            boolean armDown = gamepad1.b;
            boolean armStop = gamepad1.x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (sliderUp > 0 && sliderDown == 0) {
                sliderMotor.setPower(sliderUp);
            } else if (sliderDown < 0 && sliderUp == 0) {
                sliderMotor.setPower(sliderDown);
            }

            if (dropperUp) {
                dropperServo.setPosition(1.0);
            } else if (dropperDown) {
                dropperServo.setPosition(0.5);
            }

            if (armUp) {
                armMotor.setPower(0.75);
            } else if (armDown) {
                armMotor.setPower(-0.75);
            } else if (armStop) {
                armMotor.setPower(0);
            }

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("y", "%5.2f", y);
            telemetry.addData("x", "%5.2f", x);
            telemetry.addData("rx", "%5.2f", rx);
            telemetry.addData("sliderUp", "%5.2f", sliderUp);
            telemetry.addData("sliderDown", "%5.2f", sliderDown);
            telemetry.addData("dropperDown", dropperUp);
            telemetry.addData("dropperUp", dropperDown);
            telemetry.addData("dropperPosition", dropperServo.getPosition());
            telemetry.addData("armUp", armUp);
            telemetry.addData("armDown", armDown);
            telemetry.addData("armStop", armStop);
            telemetry.update();
        }
    }
}
