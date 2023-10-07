package org.firstinspires.ftc.teamcode.Richard;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp

public class RichardMyTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        timer.reset();

        boolean wasPressed = false;
        while (opModeIsActive()) {
            boolean isPressed = gamepad1.a;
            if (isPressed && !wasPressed) {
                timer.reset();
            }

            wasPressed = isPressed;

            double rounded = Math.round(timer.time() * 100) / 100;

            telemetry.addLine(String.valueOf(rounded));
            telemetry.update();
        }
    }
}

