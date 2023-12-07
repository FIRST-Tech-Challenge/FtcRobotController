package org.firstinspires.ftc.teamcode.robots.opmode.botb;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.LocalizationTest;
import org.firstinspires.ftc.teamcode.robots.BotB2023;

@TeleOp(group = "Robot B", name = "Robot B - LocalizationTest")
public class BotBLocalizationTest extends LocalizationTest {
    @Override
    public void runOpMode() throws InterruptedException {
        BotB2023 drive = new BotB2023(hardwareMap);
        opModeCode(drive, -1, -1, 1);
    }
}
