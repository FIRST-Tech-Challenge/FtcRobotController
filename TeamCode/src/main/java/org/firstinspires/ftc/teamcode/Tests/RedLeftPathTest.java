package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
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
    Waypoint start =
        new StartWaypoint(
            new com.arcrobotics.ftclib.geometry.Pose2d(-38.5, -62, new Rotation2d(toRadians(-90))));
    EndWaypoint spike = new EndWaypoint(-55,-32, toRadians(-160), 0.7,0.7, 15,5,toRadians(10));

    Path dropPath = new Path(start, spike);
        Path toTruss = new Path();
        toTruss.add(new StartWaypoint(dropPath.get(dropPath.size()-1).getPose()));
    toTruss.add(new EndWaypoint(-40, -59, toRadians(180), 0.5,0.8, 15, 4,toRadians(10)));
        Path toBackdrop = new Path();
        toBackdrop.add(new StartWaypoint(toTruss.get(toTruss.size()-1).getPose()));
        toBackdrop.add(new GeneralWaypoint(20,-57, Math.toRadians(180), 0.5,0.8,30));
        toBackdrop.add(new EndWaypoint(50, -36, toRadians(180), 0.5, 0.8, 30, 4, toRadians(10)));

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
            robot.followPPPath(dropPath);
            robot.followPPPath(toTruss);
            robot.followPPPath(toBackdrop);
            robot.update();
        }
        robot.stop();
    }
}
