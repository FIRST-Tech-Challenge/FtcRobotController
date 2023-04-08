package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
//@Disabled
@Config
@Autonomous(name = "OpenClaw")


public class OpenClaw extends LinearOpMode {

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, true);

        waitForStart();

        if (isStopRequested()) return;



        robot.stop();
        if (getRuntime() > 29.8) {
            stop();
        }
    }
}
