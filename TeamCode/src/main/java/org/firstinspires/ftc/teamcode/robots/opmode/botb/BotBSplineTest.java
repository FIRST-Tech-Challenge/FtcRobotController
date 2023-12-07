package org.firstinspires.ftc.teamcode.robots.opmode.botb;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.SplineTest;
import org.firstinspires.ftc.teamcode.robots.BotA2023;
import org.firstinspires.ftc.teamcode.robots.BotB2023;

@Autonomous(group = "Robot B", name = "Robot B - SplineTest")
public class BotBSplineTest extends SplineTest {

        @Override
        public void runOpMode() throws InterruptedException {
            BotB2023 drive = new BotB2023(hardwareMap);
            super.opModeCode(drive);
        }
}
