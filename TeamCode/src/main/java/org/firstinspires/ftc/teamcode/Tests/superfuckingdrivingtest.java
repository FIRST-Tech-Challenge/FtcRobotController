package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

@Autonomous(name="superfuckingdrivingtest")
public class superfuckingdrivingtest extends LinearOpMode {
    CompBotW1Attachments r = new CompBotW1Attachments();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        while(!isStopRequested()) {
            r.fl.setPower(1);
            r.fr.setPower(-1);
            r.bl.setPower(-1);
            r.br.setPower(1);
        }
        r.stop();
    }
}
