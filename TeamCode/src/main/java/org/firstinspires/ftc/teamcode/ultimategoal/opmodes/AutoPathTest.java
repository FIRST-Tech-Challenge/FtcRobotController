package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;

@Autonomous
public class AutoPathTest extends LinearOpMode {

    public Robot robot;

    PathFollow pf1;
    PathFollow pf2;

    public void runOpMode() {

        robot = new Robot(hardwareMap, telemetry, this);

        pf1 = new PathFollow( new Point[]{
                        new Point(0,0),
                        new Point(24,24),
                        new Point(24,48)
                }, robot, "test1"
        );

        pf2 = new PathFollow( new Point[]{
                        new Point(24,48),
                        new Point(24,24),
                        new Point(0,0)
                }, robot, "test2"
        );

        waitForStart();

        robot.startModules();

        pf1.pathFollow(0, 0.8, 0.8, true, 0);
        pf2.pathFollow(Math.PI, 0.8, 0.8, true, 0);
    }
}

