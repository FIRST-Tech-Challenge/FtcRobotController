package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.bot.Bot;
import org.firstinspires.ftc.teamcode.bot.BotProperties;

/**
 * The one TeleOp. There is no red or blue version.
 *
 * <p>This OpMode builds the {@link Bot} and runs the loop. The Bot does the rest: each
 * loop, every part of the robot reads the gamepads, then every part does its work.
 *
 * <p>At INIT, point the robot away from the driver. That direction becomes "forward"
 * for field-centric drive. Press circle during the match to re-set it.
 */
@TeleOp(name = "HyDrive")
public class HyDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        BotProperties properties = new BotProperties();
        // The launcher is not part of HyDrive yet (docs/decisions/0009).
        properties.launcherDisabled = true;

        Bot bot = new Bot(this, properties);
        bot.Init();

        // Waiting for START. Shows Safe Mode and the heading, so they can be checked.
        while (opModeInInit()) {
            bot.StartLoop();
            bot.ShowStatus();
            bot.EndLoop();
        }

        // START was pressed. This loop runs about 50 times a second until STOP.
        while (opModeIsActive()) {
            bot.StartLoop();
            bot.HandleUserInput();
            bot.Process();
            bot.EndLoop();
        }
    }
}
