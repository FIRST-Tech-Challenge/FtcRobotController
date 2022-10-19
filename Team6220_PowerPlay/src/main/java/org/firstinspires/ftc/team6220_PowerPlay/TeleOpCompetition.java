package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpCompetition", group = "Competition")

public class TeleOpCompetition extends BaseTeleOp {

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            driveHolo(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            idle();
        }
    }
}
