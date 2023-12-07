package org.firstinspires.ftc.teamcode.robots.opmode.bota;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.LocalizationTest;
import org.firstinspires.ftc.teamcode.robots.BotA2023;

@TeleOp(group = "Robot A", name = "Robot A - LocalizationTest")
public class BotALocalizationTest extends LocalizationTest {

    @Override
    public void runOpMode() throws InterruptedException {
        BotA2023 drive = new BotA2023(hardwareMap);
        opModeCode(drive, 1, 1, 1);
    }
}
