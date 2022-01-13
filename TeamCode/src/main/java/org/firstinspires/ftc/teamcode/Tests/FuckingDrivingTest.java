package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

@Autonomous(name="fuck me")
public class FuckingDrivingTest extends LinearOpMode {
    CompBotW1Attachments r = new CompBotW1Attachments();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);

        waitForStart();
        r.AEncDrive(0,36,0,0.5);
    }
}
