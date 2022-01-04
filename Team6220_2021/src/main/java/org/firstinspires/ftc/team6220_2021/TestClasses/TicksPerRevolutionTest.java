package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_2021.MasterOpMode;

@TeleOp(name = "Ticks Per Revolution", group = "TeleOp")
@Disabled
public class TicksPerRevolutionTest extends MasterOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();

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