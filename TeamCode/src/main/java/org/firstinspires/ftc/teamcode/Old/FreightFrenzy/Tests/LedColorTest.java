package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware.LedColor;@Disabled

@Autonomous(name = "Led Color Test")

public class LedColorTest extends LinearOpMode {


    /* local OpMode members. */

    @Override
    public void runOpMode() {
        LedColor led = new LedColor(this);
        long sleepTime = 1000;
        waitForStart();

        for (int i=1; i<=4;i++) {
            led.LedRed(i);
            sleep(sleepTime);
            led.LedGreen(i);
            sleep(sleepTime);
            led.LedAmber(i);
            sleep(sleepTime);
            led.LedOff(i);
            sleep(sleepTime);

        }

    }

}
