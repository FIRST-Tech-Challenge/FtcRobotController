package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Driver Control", group="XDrive")
public class DriverControl extends LinearOpMode {
    DriveTrain bot;

    @Override
    public void runOpMode() {

        bot.init();
        bot.setControlVariables(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        waitForStart();

        sleep(1000);

        while (opModeIsActive())
        {
            bot.loop();

            if (bot.isTurning) { bot.turn(); }

            if (!bot.isTurning) { bot.move(); }
        }
        bot.stop();
    }
}
