package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TeleOpSus", group = "Robot")
public class TeleOpSuspension extends LinearOpMode {

    private Suspension suspension;

    @Override
    public void runOpMode() {
        suspension = new Suspension(this);
        waitForStart();
        while (opModeIsActive()){

            if (gamepad1.y) {
                suspension.moveUp();

            } else if (gamepad1.a) {
                suspension.moveDown();

            } else {
                suspension.stop();

            }

        }
    }
}
