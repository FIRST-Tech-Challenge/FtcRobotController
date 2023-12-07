package org.firstinspires.ftc.teamcode.robots.opmode.botb;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.TrackWidthTuner;
import org.firstinspires.ftc.teamcode.robots.BotA2023;
import org.firstinspires.ftc.teamcode.robots.BotB2023;

@Autonomous(group = "Robot B", name = "Robot B - TrackWidthTuner")
public class BotBTrackWidthTuner extends TrackWidthTuner {

            @Override
            public void runOpMode() throws InterruptedException {
                BotB2023 drive = new BotB2023(hardwareMap);
                super.opModeCode(drive);
            }
}
