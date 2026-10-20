package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.bot.Bot;
import org.firstinspires.ftc.teamcode.bot.BotProperties;

/**
 * Camera check. Nothing moves.
 *
 * <p>Point the camera at an AprilTag. The Driver Station shows everything the robot knows
 * about the nearest one: its id, how many degrees left or right it is, how far away, and
 * how long ago it was last seen. With a webcam, the Driver Station's Camera Stream (three
 * dots menu) shows the picture too.
 *
 * <p>Only the camera runs. The drive and the launcher are switched off.
 */
@TeleOp(name = "Test: Vision", group = "Test")
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        BotProperties properties = new BotProperties();
        properties.driveDisabled = true;
        properties.launcherDisabled = true;

        Bot bot = new Bot(this, properties);
        bot.Init();

        while (opModeInInit()) {
            bot.StartLoop();
            bot.ShowStatus();
            telemetry.addLine("Point the camera at an AprilTag.");
            bot.EndLoop();
        }

        while (opModeIsActive()) {
            bot.StartLoop();
            bot.Process();
            bot.EndLoop();
        }
    }
}
