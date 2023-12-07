package org.firstinspires.ftc.teamcode.robots.opmode.botb;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.ManualFeedforwardTuner;
import org.firstinspires.ftc.teamcode.robots.BotA2023;
import org.firstinspires.ftc.teamcode.robots.BotB2023;

@Autonomous(group = "Robot B", name = "Robot B - ManualFeedForwardTuner")
public class BotBManualFeedForwardTuner extends ManualFeedforwardTuner {

        @Override
        public void runOpMode() {
            drive = new BotB2023(hardwareMap);
            super.opModeCode();
        }
}
