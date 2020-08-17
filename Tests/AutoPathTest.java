package org.firstinspires.ftc.teamcode.rework.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;
import org.firstinspires.ftc.teamcode.rework.Robot;

import java.util.ArrayList;

@Autonomous
public class AutoPathTest extends LinearOpMode {

    Robot robot;

    public void runOpMode() {

        initRobot();

        ArrayList<Waypoint> path = new ArrayList<Waypoint>();
        path.add(new Waypoint(0,0));
        path.add(new Waypoint(0,60));
        path.add(new Waypoint(-60,60));
        path.add(new Waypoint(-60,0));

        waitForStart();
        robot.startModules();

        while (opModeIsActive()) {

            robot.movements.pathFollow(path,0.8,0.8);
            sleep(5000);

            break;
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry,this);
        robot.initModules();
    }
}

