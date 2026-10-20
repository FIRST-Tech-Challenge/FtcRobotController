package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.bot.Bot;
import org.firstinspires.ftc.teamcode.bot.BotProperties;

/**
 * Launcher speed test. Wheel guard on, hands clear, hand on STOP.
 *
 * <p>Gamepad 1:
 * <ul>
 *   <li><b>Hold right trigger</b>: spin the launcher at the target speed.
 *   <li><b>Right bumper / left bumper</b>: target speed up / down 50 RPM.
 * </ul>
 *
 * <p>Only the launcher runs. The drive is switched off, so the wheels never move.
 *
 * <p>The target starts at 4000 RPM in this test. The Driver Station shows target and
 * actual RPM, so you can see how close the speed control gets. See docs/decisions/0009
 * for tuning.
 */
@TeleOp(name = "Test: Launcher", group = "Test")
public class LauncherMotorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        BotProperties properties = new BotProperties();
        // This test is about the launcher. Leave the drive out of it.
        properties.driveDisabled = true;
        // Start faster than the normal 1000 RPM. Only this OpMode is changed.
        properties.launcherStartRpm = 4000;

        Bot bot = new Bot(this, properties);
        bot.Init();

        while (opModeInInit()) {
            bot.StartLoop();
            bot.ShowStatus();
            telemetry.addLine("Hold right trigger to spin. Bumpers change the speed.");
            bot.EndLoop();
        }

        while (opModeIsActive()) {
            bot.StartLoop();
            bot.HandleUserInput();
            bot.Process();
            bot.EndLoop();
        }
    }
}
