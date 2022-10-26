package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpTest", group = "Test")

public class TeleOpCompetition extends BaseTeleOp{

    @Override public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {

            driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            /* telemetry.addData("motFL", "%.1f ", motorFL.getCurrentPosition());
            telemetry.addData("motFR", "%.1f ", motorFR.getCurrentPosition());
            telemetry.addData("motBL", "%.1f ", motorBL.getCurrentPosition());
            telemetry.addData("motBR", "%.1f ", motorBR.getCurrentPosition());
            telemetry.update();
             */

            idle();

        }
    }
}
