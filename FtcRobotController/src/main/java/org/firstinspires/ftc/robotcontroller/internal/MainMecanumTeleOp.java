package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleLEDPatternResponse;
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
        DcMotor pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        DcMotor sliderMotor = hardwareMap.dcMotor.get("sliderMotor");
        CRServo pickUpServo = hardwareMap.crservo.get("intakeServo");
        Servo rotateIntake = hardwareMap.servo.get("rotateServo");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counter = 1;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;
            boolean pivotUp = gamepad1.right_bumper;
            boolean pivotDown = gamepad1.left_bumper;
            double sliderUp = -gamepad1.right_trigger;
            double sliderDown = gamepad1.left_trigger;
            boolean pickUp = gamepad1.a;
            boolean pickDown = gamepad1.b;
            boolean pickStop = gamepad1.x;
            boolean resetIntake = gamepad1.y;
            boolean turnLeft = gamepad1.dpad_left;
            boolean turnRight = gamepad1.dpad_right;
            boolean turnCenter = gamepad1.dpad_down;


            /*
            * Forward/Backward = Left Joystick Y
            * Strafe Left/Right = Left Joystick X
            * Turn Left/Right = Right Joystick X
            * Slider Up = Right Trigger
            * Slider Down = Left Trigger
            * Pivot Up = Right Bumper
            * Pivot Down = Left Bumper
            * Intake = A
            * Outtake = B
            * StopRotate = X
            * resetIntake = Y
            * Intake Rotate Left = D-Pad Left;
            * Intake Rotate Right = D-Pad Right;
            * Intake Center = D-Pad Down
            * */

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator) / 1.5;
            double frontRightPower = ((y - x - rx) / denominator) / 1.5;
            double backLeftPower = ((y - x + rx) / denominator) / 1.5;
            double backRightPower = ((y + x - rx) / denominator) / 1.5;

            // Slider Initialization

            if (counter == 1) {
                sliderMotor.setTargetPosition(200);
                sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double sliderMotorPos = sliderMotor.getCurrentPosition();
                while (sliderMotorPos < 200) {
                    sliderMotorPos = sliderMotor.getCurrentPosition();
                    sliderMotor.setPower(0.4);

                    telemetry.addLine("Slider Initialization:");
                    telemetry.addData("  Current Position", sliderMotorPos);
                    telemetry.addData("  Target Position", 200);
                    telemetry.update();
                }
            }

            // Check if sliderDown is greater than 0 before setting the power
            if (sliderDown > 0) {
                sliderMotor.setPower(sliderDown);
            } else if (sliderUp < 0) {
                if (sliderMotor.getCurrentPosition() > 2100) {
                    sliderMotor.setPower(0);
                }
                sliderMotor.setPower(sliderUp);
            }

            if (pickUp) {
                pickUpServo.setPower(0.5); // Change later
            } else if (pickDown) {
                pickUpServo.setPower(-0.5); // Change later
            } else if (pickStop) {
                pickUpServo.setPower(0);
            }

            if (turnLeft) {
                rotateIntake.setPosition(0.0);
            } else if (turnRight) {
                rotateIntake.setPosition(1.0);
            } else if (turnCenter) {
                rotateIntake.setPosition(0.6);
            }

            if (pivotUp) {
                pivotMotor.setTargetPosition(-3120);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(0.5);
            } else if (pivotDown) {
                pivotMotor.setTargetPosition(-100);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(-0.5);
            }

            if (resetIntake) {
                pivotMotor.setTargetPosition(-100);
                sliderMotor.setTargetPosition(-200);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(-0.5);
                sliderMotor.setPower(0.4);
            }

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addLine("Centralized Telemetry:");
            telemetry.addData("  Left Stick Y", y);
            telemetry.addData("  Left Stick X", x);
            telemetry.addData("  Right Stick X", rx);
            telemetry.addData("  Right Trigger", sliderUp);
            telemetry.addData("  Left Trigger", sliderDown);
            telemetry.addLine("Motor Encoders:");
            telemetry.addData("  Front Left", frontLeftMotor.getCurrentPosition());
            telemetry.addData("  Front Right", frontRightMotor.getCurrentPosition());
            telemetry.addData("  Back Left", backLeftMotor.getCurrentPosition());
            telemetry.addData("  Back Right", backRightMotor.getCurrentPosition());
            telemetry.addData(" Pivot", pivotMotor.getCurrentPosition());
            telemetry.addData(" Slider", sliderMotor.getCurrentPosition());
            telemetry.addLine("Servo Positions:");
            telemetry.addData("  Rotate Intake", rotateIntake.getPosition());
            telemetry.addData(" Down", turnCenter);
            telemetry.update();

            counter += 1;
        }
    }
}