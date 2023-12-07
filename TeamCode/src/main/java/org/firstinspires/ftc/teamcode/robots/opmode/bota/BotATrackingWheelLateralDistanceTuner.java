package org.firstinspires.ftc.teamcode.robots.opmode.bota;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.TrackingWheelLateralDistanceTuner;
import org.firstinspires.ftc.teamcode.robots.BotA2023;
import org.firstinspires.ftc.teamcode.robots.BotB2023;

@TeleOp(group = "Robot A", name = "Robot A - TrackingWheelLateralDistanceTuner")

public class BotATrackingWheelLateralDistanceTuner extends TrackingWheelLateralDistanceTuner {

        @Override
        public void runOpMode() throws InterruptedException {
            BotA2023 drive = new BotA2023(hardwareMap);
            super.opModeCode(drive);
        }
}
