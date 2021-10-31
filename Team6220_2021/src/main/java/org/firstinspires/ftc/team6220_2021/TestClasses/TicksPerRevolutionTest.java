package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Ticks Per Revolution", group = "TeleOp")
public class TicksPerRevolutionTest extends LinearOpMode {
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;

    @Override
    public void runOpMode() throws InterruptedException {
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);
        motorFrontLeft.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double degree = 0;
        double input = 0;

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                input = 537.6 * (18/4);
                int value = (int)input;
                motorFrontLeft.setTargetPosition(-value);
                motorFrontRight.setTargetPosition(value);
                motorBackRight.setTargetPosition(value);
                motorBackLeft.setTargetPosition(-value);

                motorFrontRight.setPower(0.7);
                motorFrontLeft.setPower(0.7);
                motorBackRight.setPower(0.7);
                motorBackLeft.setPower(0.7);
            }
            else if (gamepad1.dpad_right) {
                input = 537.6 * (18/16);
                int value = (int)input;
                while (motorFrontRight.getCurrentPosition()<value && opModeIsActive())
                {
                    motorFrontLeft.setTargetPosition(value);
                    motorFrontRight.setTargetPosition(value);
                    motorBackRight.setTargetPosition(value);
                    motorBackLeft.setTargetPosition(value);

                    motorFrontRight.setPower(-0.7);
                    motorFrontLeft.setPower(0.7);
                    motorBackRight.setPower(-0.7);
                    motorBackLeft.setPower(0.7);
                }
            }
            else if (gamepad1.dpad_down) {
                input = 537.6 * (18/8);
                int value = (int)input;
                motorFrontLeft.setTargetPosition(value);
                motorFrontRight.setTargetPosition(value);
                motorBackRight.setTargetPosition(value);
                motorBackLeft.setTargetPosition(value);

                motorFrontRight.setPower(-0.7);
                motorFrontLeft.setPower(0.7);
                motorBackRight.setPower(-0.7);
                motorBackLeft.setPower(0.7);
            }
            else if (gamepad1.dpad_left) {
                input = 537.6 * (18/12);
                int value = (int)input;
                motorFrontLeft.setTargetPosition(value);
                motorFrontRight.setTargetPosition(value);
                motorBackRight.setTargetPosition(value);
                motorBackLeft.setTargetPosition(value);

                motorFrontRight.setPower(-0.7);
                motorFrontLeft.setPower(0.7);
                motorBackRight.setPower(-0.7);
                motorBackLeft.setPower(0.7);
            }
        }
    }
}