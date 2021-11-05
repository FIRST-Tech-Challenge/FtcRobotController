package org.firstinspires.ftc.teamcode.CompBotSimplified;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AssistedEncoderDrive extends LinearOpMode {
    CompBotHWSimplified r = new CompBotHWSimplified();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();

        // Drive 12 in at 45 degree angle
        r.assistedEncoderDrive(48/Math.sqrt(2), 12/Math.sqrt(2), 0.5, 0.5);
    }
}
