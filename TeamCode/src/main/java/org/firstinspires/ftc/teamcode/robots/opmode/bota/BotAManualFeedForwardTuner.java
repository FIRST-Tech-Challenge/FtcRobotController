package org.firstinspires.ftc.teamcode.robots.opmode.bota;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.ManualFeedforwardTuner;
import org.firstinspires.ftc.teamcode.robots.BotA2023;

@Autonomous(group = "Robot A", name = "Robot A - ManualFeedForwardTuner")
public class BotAManualFeedForwardTuner extends ManualFeedforwardTuner {

        @Override
        public void runOpMode() {
            drive = new BotA2023(hardwareMap);
            super.opModeCode();
        }
}
