package org.firstinspires.ftc.teamcode.rework.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;
import org.firstinspires.ftc.teamcode.rework.Robot;
import org.firstinspires.ftc.teamcode.rework.RobotTools.PathFollow;

import java.util.ArrayList;

@Autonomous
public class AutoPathTest extends LinearOpMode {

    public Robot robot;

    PathFollow pf1;
    PathFollow pf2;

    public void runOpMode() {

        robot = new Robot(hardwareMap, telemetry, this);
        robot.initModules();

        pf1 = new PathFollow( new Waypoint[]{
                        new Waypoint(0,0),
                        new Waypoint(24,24),
                        new Waypoint(24,48)
                }, robot, "test1"
        );

        pf2 = new PathFollow( new Waypoint[]{
                        new Waypoint(24,48),
                        new Waypoint(24,24),
                        new Waypoint(0,0)
                }, robot, "test2"
        );

        waitForStart();

        robot.startModules();

        pf1.pathFollow(0, 0.8, 0.8, true, 0);
        pf2.pathFollow(Math.PI, 0.8, 0.8, true, 0);
    }
}

