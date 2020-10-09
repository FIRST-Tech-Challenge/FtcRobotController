package org.firstinspires.ftc.teamcode.ultimategoal.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

@Autonomous
public class SimpleStraightTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);

        PathFollow pf1 = new PathFollow(new Point[]{
                new Point(0, 0),
                new Point(0, 24),
                new Point(0, 48)
        }, robot, "test");

        waitForStart();

        robot.startModules();

        pf1.pathFollow(0, 1, 1, false, 0);

        sleep(5000);
    }
}
