package org.firstinspires.ftc.teamcode.rework.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;
import org.firstinspires.ftc.teamcode.rework.Robot;

import java.util.ArrayList;

@Autonomous
public class PathPIDTest extends LinearOpMode {

    Robot robot;

    public void runOpMode() {

        initRobot();

        ArrayList<Waypoint> path = new ArrayList<Waypoint>();
        path.add(new Waypoint(0, 0));
        path.add(new Waypoint(0, 30));

        waitForStart();
        robot.startModules();

        while (opModeIsActive()) {

            //robot.movements.pathFollow(path, 0, 0.8, 0.8, true, Math.PI*-0.5);

            break;
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry, this);
        robot.initModules();
    }
}

