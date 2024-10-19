package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto Main Mecanum Drive")
public class MecaumAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor sliderMotor = hardwareMap.dcMotor.get("sliderMotor");
        DcMotor pickUpMotor = hardwareMap.dcMotor.get("pickUpMotor");
        Servo dropperServoRight = hardwareMap.servo.get("dropperServoRight");
        Servo dropperServoLeft = hardwareMap.servo.get("dropperServoLeft");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int forward = -1000;
        int backward = 1000;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;
            double sliderUp = gamepad1.right_trigger;
            double sliderDown = -gamepad1.left_trigger;
            boolean dropperDown = gamepad1.dpad_down;
            boolean dropperUp = gamepad1.dpad_up;
            boolean pickUp = gamepad1.a;
            boolean pickDown = gamepad1.b;
            boolean pickStop = gamepad1.x;

//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            if (sliderUp > 0 && sliderDown == 0) {
//                sliderMotor.setPower(sliderUp);
//            } else if (sliderDown < 0 && sliderUp == 0) {
//                sliderMotor.setPower(sliderDown);
//            }
//
//            if (dropperUp) {
//                dropperServoRight.setPosition(0.25);
//                dropperServoLeft.setPosition(0.25);
//            } else if (dropperDown) {
//                dropperServoRight.setPosition(-0.42);
//                dropperServoLeft.setPosition(-0.42);
//            }
//
//            if (pickUp) {
//                pickUpMotor.setPower(-0.75);
//            } else if (pickDown) {
//                pickUpMotor.setPower(0.75);
//            } else if (pickStop) {
//                pickUpMotor.setPower(0);
//            }
//
//            if (gamepad1.right_stick_button) {
//
//            }

//            frontLeftMotor.setPower(frontLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backLeftMotor.setPower(backLeftPower);
//            backRightMotor.setPower(backRightPower);

            boolean direction = true;

            if (direction) {
                frontLeftMotor.setTargetPosition(forward);
                frontRightMotor.setTargetPosition(forward);
                backLeftMotor.setTargetPosition(forward);
                backRightMotor.setTargetPosition(forward);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0.5);
                backLeftMotor.setPower(0.5);
                backRightMotor.setPower(0.5);

                if (frontLeftMotor.getCurrentPosition() >= -995 && frontRightMotor.getCurrentPosition() >= -995 && backLeftMotor.getCurrentPosition() >= -995 && backRightMotor.getCurrentPosition() >= -995) {
                    direction = false;
                }
            } else {
                frontLeftMotor.setTargetPosition(backward);
                frontRightMotor.setTargetPosition(backward);
                backLeftMotor.setTargetPosition(backward);
                backRightMotor.setTargetPosition(backward);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftMotor.setPower(-0.5);
                frontRightMotor.setPower(-0.5);
                backLeftMotor.setPower(-0.5);
                backRightMotor.setPower(-0.5);

                if (frontLeftMotor.getCurrentPosition() >= 995 && frontRightMotor.getCurrentPosition() >= 995 && backLeftMotor.getCurrentPosition() >= 995 && backRightMotor.getCurrentPosition() >= 995) {
                    direction = true;
                }
            }

//            telemetry.addData("y", "%5.2f", y);
//            telemetry.addData("x", "%5.2f", x);
//            telemetry.addData("rx", "%5.2f", rx);
//            telemetry.addData("sliderUp", "%5.2f", sliderUp);
//            telemetry.addData("sliderDown", "%5.2f", sliderDown);
//            telemetry.addData("dropperDown", dropperUp);
//            telemetry.addData("dropperUp", dropperDown);
//            telemetry.addData("dropperPosition", dropperServoRight.getPosition());
//            telemetry.addData("pickUp", pickUp);
//            telemetry.addData("pickDown", pickDown);
//            telemetry.addData("pickStop", pickStop);
            telemetry.addData("frontLeftMotor", frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightMotor", frontRightMotor.getCurrentPosition());
            telemetry.addData("backLeftMotor", backLeftMotor.getCurrentPosition());
            telemetry.addData("backRightMotor", backRightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
