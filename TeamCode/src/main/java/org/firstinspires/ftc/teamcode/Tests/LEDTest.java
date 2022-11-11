package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.Components.LEDStrip;
import org.firstinspires.ftc.teamcode.Old.Components.Localizer.OdometryTracker;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@TeleOp
public class LEDTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        LEDStrip led = new LEDStrip();



        while (!isStopRequested()) {
            led.blue();
        }
    }
}
