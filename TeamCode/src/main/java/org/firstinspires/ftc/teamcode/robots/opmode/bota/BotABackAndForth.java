package org.firstinspires.ftc.teamcode.robots.opmode.bota;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.BackAndForth;
import org.firstinspires.ftc.teamcode.robots.BotA2023;

@Autonomous(group = "Robot A", name = "Robot A - BackAndForth")
public class BotABackAndForth extends BackAndForth {

            @Override
            public void runOpMode() throws InterruptedException {
                BotA2023 drive = new BotA2023(hardwareMap);
                super.opModeCode(drive);
            }
}
