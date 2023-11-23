package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedLeftPathTest")
public class RedLeftPathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        robot.roadrun.setPoseEstimate(new Pose2d(-38.5, -62, Math.toRadians(-90)));
        TrajectorySequence tradegy = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -62, Math.toRadians(-90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-55, -32, toRadians(-160)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-40, -58, toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(5, -57.5), toRadians(10))
                .splineTo(new Vector2d(50, -28), toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(5, -57.5), toRadians(185))
                .splineTo(new Vector2d(-20, -55.5), toRadians(175))
                .splineTo(new Vector2d(-57, -37.5), toRadians(165))
                .setReversed(true)
                .splineTo(new Vector2d(-25, -55.5), toRadians(-10))
                .splineTo(new Vector2d(5, -55.5), toRadians(10))
                .splineTo(new Vector2d(50, -44), toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(5, -57.5), toRadians(185))
                .splineTo(new Vector2d(-20, -55.5), toRadians(175))
                .splineTo(new Vector2d(-57, -37.5), toRadians(165))
                .setReversed(true)
                .splineTo(new Vector2d(-22, -55.5), toRadians(-10))
                .splineTo(new Vector2d(5, -55.5), toRadians(10))
                .splineTo(new Vector2d(50, -44), toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(5, -55.5), toRadians(185))
                .splineTo(new Vector2d(-20, -55.5), toRadians(175))
                .splineTo(new Vector2d(-57, -28), toRadians(155))
                .setReversed(true)
                .splineTo(new Vector2d(-22, -55.5), toRadians(-10))
                .splineTo(new Vector2d(5, -55.5), toRadians(10))
                .splineTo(new Vector2d(50, -44), toRadians(0))
        .build();
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()&&!robot.queuer.isFullfilled()){
            robot.followTrajSeq(tradegy);
            robot.update();
        }
        robot.stop();
    }
}
