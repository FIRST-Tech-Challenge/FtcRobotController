package org.firstinspires.ftc.teamcode.robots.opmode.teamgrantbot2023;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.LocalizationTest;
import org.firstinspires.ftc.teamcode.robots.TeamGrantBot2023;

@TeleOp(group = "TeamGrantBot2023", name = "TeamGrant LocalizationTest")
public class TGLocalizationTest extends LocalizationTest {

    @Override
    public void runOpMode() throws InterruptedException {
        TeamGrantBot2023 drive = new TeamGrantBot2023(hardwareMap);
        opModeCode(drive);
    }
}
