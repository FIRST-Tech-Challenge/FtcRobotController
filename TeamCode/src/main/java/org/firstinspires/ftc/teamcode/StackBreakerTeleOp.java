package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpSB", group = "Robot")

public class StackBreakerTeleOp extends LinearOpMode {
    private StackBreaker stackBreaker;

    private boolean stateX = false;

    @Override
    public void runOpMode() {
        stackBreaker = new StackBreaker(this);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.x && !stateX) {
                stackBreaker.switchPosition();
            }
            stateX = gamepad1.x;
        }
    }
}
