package org.firstinspires.ftc.teamcode.CompBotV3;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous
public class EncoderDriveVerification extends LinearOpMode {
    CompBotV3Attachments r = new CompBotV3Attachments();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);

        waitForStart();

        r.AEncDrive(0,10,0,0.2);

    }
}
