package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.TeleopBot;

@Config
@TeleOp(name = "TeleopDrivetrainTest", group = "Test")

public class TeleopDrivetrainTest extends LinearOpMode {

    private TeleopBot bot;

    public static boolean loggingOn = true;

    @Override
    public void runOpMode() {

        double driveAxial = 0.0;
        double driveStrafe = 0.0;
        double driveYaw = 0.0;

        bot = new TeleopBot(hardwareMap, telemetry, loggingOn);
        waitForStart();

        while (opModeIsActive() && !gamepad1.ps) {

            if (gamepad1.dpad_up) {
                bot.creepDirection(1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_down) {
                bot.creepDirection(-1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_left) {
                bot.creepDirection(0.0, 1.0, 0.0);
            } else if (gamepad1.dpad_right) {
                bot.creepDirection(0.0, -1.0, 0.0);
            } else {
                driveAxial = gamepad1.left_stick_y;
                driveStrafe = gamepad1.left_stick_x;
                driveYaw = gamepad1.right_stick_x;
                if ((Math.abs(driveAxial) < 0.2) && (Math.abs(driveStrafe) < 0.2) && (Math.abs(driveYaw) < 0.2)) {
                    bot.stopDrive();
                } else
                    bot.moveDirection(-driveAxial, -driveStrafe, -driveYaw);
            }

            bot.update();
        }
    }
}
