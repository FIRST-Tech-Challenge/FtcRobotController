package org.firstinspires.ftc.teamcode.robots.opmode.botb;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.TrackingWheelLateralDistanceTuner;
import org.firstinspires.ftc.teamcode.robots.BotB2023;

@TeleOp(group = "Robot B", name = "Robot B - TrackingWheelLateralDistanceTuner")

public class BotBTrackingWheelLateralDistanceTuner extends TrackingWheelLateralDistanceTuner {

        @Override
        public void runOpMode() throws InterruptedException {
            BotB2023 drive = new BotB2023(hardwareMap);
            super.opModeCode(drive);
        }
}
