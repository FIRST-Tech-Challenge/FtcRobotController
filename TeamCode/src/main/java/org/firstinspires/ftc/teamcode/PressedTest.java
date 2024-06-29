package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class PressedTest extends LinearOpMode {
    //hardwaremap here
    @Override
    public void runOpMode() {

        waitForStart();

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        boolean pressedLastIteration = false;

        while (opModeIsActive()) {

            boolean gamepad1A_pressed = gamepad1.a;
            boolean gamepad1B_pressed = gamepad1.b;

            if (buttonHandler.isPressedOnceA(gamepad1A_pressed)) {
                //DO STUFF
            }
            if (gamepad1B_pressed & !pressedLastIteration) {
                //DO STUFF
            }

            pressedLastIteration = gamepad1A_pressed;



        }

    }
}
