package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

@Config
@Autonomous(name = "Crash")


public class Crash extends LinearOpMode {
    public void runOpMode() {
//        PwPRobot robot = new PwPRobot(this, false);
        waitForStart();
        while (true) {
            telemetry.addData("stop program ;)", 8008135);
            telemetry.update();
        }
    }
}
