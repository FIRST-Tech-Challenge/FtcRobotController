package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

@Autonomous(name="a")
public class a extends LinearOpMode {
    CompBotW1Attachments r = new CompBotW1Attachments();
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        ElapsedTime e = new ElapsedTime();

        while(e.milliseconds() < 1000) {
            r.setLiftPower(1);
        }

        e.reset();

        while(e.milliseconds() < 1000) {
            r.poweredHoldCycle();
        }

        r.stopPoweredHold();

        e.reset();

        while(e.milliseconds() < 1000) {
            r.setLiftPower(-1);
        }

        r.stop();
    }
}
