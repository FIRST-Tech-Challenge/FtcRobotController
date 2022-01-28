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
                motorFL.setTargetPosition(-value);
                motorFR.setTargetPosition(value);
                motorBR.setTargetPosition(value);
                motorBL.setTargetPosition(-value);

                motorFR.setPower(0.7);
                motorFL.setPower(0.7);
                motorBR.setPower(0.7);
                motorBL.setPower(0.7);
            }
            else if (gamepad1.dpad_right) {
                input = 537.6 * (18/16);
                int value = (int)input;
                while (motorFR.getCurrentPosition()<value && opModeIsActive())
                {
                    motorFL.setTargetPosition(value);
                    motorFR.setTargetPosition(value);
                    motorBR.setTargetPosition(value);
                    motorBL.setTargetPosition(value);

                    motorFR.setPower(-0.7);
                    motorFL.setPower(0.7);
                    motorBR.setPower(-0.7);
                    motorBL.setPower(0.7);
                }
            }
            else if (gamepad1.dpad_down) {
                input = 537.6 * (18/8);
                int value = (int)input;
                motorFL.setTargetPosition(value);
                motorFR.setTargetPosition(value);
                motorBR.setTargetPosition(value);
                motorBL.setTargetPosition(value);

                motorFR.setPower(-0.7);
                motorFL.setPower(0.7);
                motorBR.setPower(-0.7);
                motorBL.setPower(0.7);
            }
            else if (gamepad1.dpad_left) {
                input = 537.6 * (18/12);
                int value = (int)input;
                motorFL.setTargetPosition(value);
                motorFR.setTargetPosition(value);
                motorBR.setTargetPosition(value);
                motorBL.setTargetPosition(value);

                motorFR.setPower(-0.7);
                motorFL.setPower(0.7);
                motorBR.setPower(-0.7);
                motorBL.setPower(0.7);
            }
        }
    }
}