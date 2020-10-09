package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;

import java.util.ArrayList;

@Autonomous
public class PathPIDTest extends LinearOpMode {

    Robot robot;

    public void runOpMode() {
        initRobot();

        ArrayList<Point> path = new ArrayList<Point>();
        path.add(new Point(0, 0));
        path.add(new Point(0, 30));

        waitForStart();
        robot.startModules();

        while (opModeIsActive()) {

            //robot.movements.pathFollow(path, 0, 0.8, 0.8, true, Math.PI*-0.5);

            break;
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry, this);
    }
}

