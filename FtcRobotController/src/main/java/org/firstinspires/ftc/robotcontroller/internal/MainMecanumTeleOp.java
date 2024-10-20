package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main Mecanum Drive")
public class MainMecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declarations

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
//        DcMotor sliderMotor = hardwareMap.dcMotor.get("sliderMotor");
        CRServo pickUpServo = hardwareMap.crservo.get("pickUpServo");
        Servo intakeRotate = hardwareMap.servo.get("turnServo");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;
            double sliderUp = gamepad1.right_trigger;
            double sliderDown = -gamepad1.left_trigger;
//            boolean dropperDown = gamepad1.dpad_down;
//            boolean dropperUp = gamepad1.dpad_up;
            boolean pickUp = gamepad1.a;
            boolean pickDown = gamepad1.b;
            boolean pickStop = gamepad1.x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator) / 1.5;
            double frontRightPower = ((y - x - rx) / denominator) / 1.5;
            double backLeftPower = ((y - x + rx) / denominator) / 1.5;
            double backRightPower = ((y + x - rx) / denominator) / 1.5;

//            if (sliderUp > 0 && sliderDown == 0) {
//                sliderMotor.setPower(sliderUp);
//            } else if (sliderDown < 0 && sliderUp == 0) {
//                sliderMotor.setPower(sliderDown);
//            }

//            if (dropperUp) {
//                dropperServoRight.setPosition(0.35);
//                dropperServoLeft.setPosition(0.35);
//            } else if (dropperDown) {
//                dropperServoRight.setPosition(-0.35);
//                dropperServoLeft.setPosition(-0.35);
//            }

            if (pickUp) {
                while (pickUp) {
                    pickUpServo.setPower(0.5); // Change later
                }
            } else if (pickDown) {
                while (pickDown) {
                    pickUpServo.setPower(-0.5); // Change later
                }
            } else if (pickStop) {
                pickUpServo.setPower(0);
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
//            telemetry.addData("dropperDown", dropperUp);
//            telemetry.addData("dropperUp", dropperDown);
//            telemetry.addData("dropperPosition", dropperServoRight.getPosition());
            telemetry.addData("pickUp", pickUp);
            telemetry.addData("pickDown", pickDown);
            telemetry.addData("pickStop", pickStop);
            telemetry.addData("frontLeftMotor", frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightMotor", frontRightMotor.getCurrentPosition());
            telemetry.addData("backLeftMotor", backLeftMotor.getCurrentPosition());
            telemetry.addData("backRightMotor", backRightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}