package org.firstinspires.ftc.teamcode.rework.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rework.AutoTools.PathFollow;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;
import org.firstinspires.ftc.teamcode.rework.Robot;

@Autonomous
public class SimpleStraightTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);

        PathFollow pf1 = new PathFollow(new Waypoint[]{
                new Waypoint(0, 0),
                new Waypoint(0, 24),
                new Waypoint(0, 48)
        }, robot, "test");

        waitForStart();

        robot.startModules();

        pf1.pathFollow(0, 1, 1, false, 0);

        sleep(5000);
    }
}
