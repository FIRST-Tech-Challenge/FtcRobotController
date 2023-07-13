package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.LEDStrip;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
@Disabled

@TeleOp(name = "LEDTest2")

public class LEDTest2 extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        PwPRobot pwprobot = new PwPRobot(this, false);
        LEDStrip led = new LEDStrip();
        waitForStart();

        while (opModeIsActive()) {

            led.violet();
        }
        stop();
    }
}
