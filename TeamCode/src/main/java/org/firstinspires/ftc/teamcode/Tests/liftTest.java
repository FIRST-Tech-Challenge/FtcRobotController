package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.nEncDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3Attachments;

@TeleOp
public class liftTest extends LinearOpMode {
    CompBotV3Attachments r = new CompBotV3Attachments();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        nEncDrive(r.lift,5000,1,telemetry);

    }
}
