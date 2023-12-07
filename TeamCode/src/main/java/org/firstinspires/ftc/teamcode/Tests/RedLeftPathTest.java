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
    robot.roadrun.setPoseEstimate(new Pose2d(-38.5, -57.5, Math.toRadians(-90)));
    Waypoint start =
        new StartWaypoint(
            new com.arcrobotics.ftclib.geometry.Pose2d(-38.5, -59, new Rotation2d(toRadians(-90))));
    EndWaypoint spike = new EndWaypoint(-55, -32, toRadians(-160), 0.8, 0.7, 15, 5, toRadians(10));

    Path dropPath = new Path(start, spike);
    Path toTruss = new Path();
    toTruss.add(new StartWaypoint(dropPath.get(dropPath.size() - 1).getPose().getTranslation()));
    toTruss.add(new EndWaypoint(-40, -58, toRadians(180), 0.8, 0.8, 15, 4, toRadians(10)));
    Path toBackdrop = new Path();
    toBackdrop.add(new StartWaypoint(toTruss.get(toTruss.size() - 1).getPose().getTranslation()));
    toBackdrop.add(new GeneralWaypoint(5, -59, toRadians(185), 1.0, 0.8, 10));
    toBackdrop.add(new EndWaypoint(45, -36, toRadians(180), 0.7, 0.8, 10, 4, toRadians(10)));
    Path backToTruss = new Path();
    backToTruss.add(
        new StartWaypoint(toBackdrop.get(toBackdrop.size() - 1).getPose().getTranslation()));
    backToTruss.add(new GeneralWaypoint(20, -59, toRadians(180), 0.8, 0.8, 5));
    backToTruss.add(new GeneralWaypoint(-35, -59, toRadians(180), 1.0, 0.8, 10));
    backToTruss.add(new EndWaypoint(-45, -34, toRadians(180), 0.8, 0.8, 15, 4, toRadians(10)));

    Path testPath = new Path();
    testPath.add(start);
    testPath.add(new EndWaypoint(-38.5, 0, Math.toRadians(-90), 0.7, 0.7, 20, 4, toRadians(10)));
    Path testPath2 = new Path();
    testPath2.add(new StartWaypoint(-38.5, 0));
    testPath2.add(new EndWaypoint(-38.5, -61, toRadians(-90), 0.7, 0.7, 20, 4, toRadians(10)));
    TrajectorySequence tradegy =
        robot
            .roadrun
            .trajectorySequenceBuilder(new Pose2d(-38.5, -62, Math.toRadians(-90)))
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
    while (opModeIsActive() && !isStopRequested() && !robot.queuer.isFullfilled()) {
//      for (int i = 0; i < 6; i++) {
//        robot.followPPPath(testPath);
//        robot.followPPPath(testPath2);
//      }
//      robot.update();
//    }
                robot.followPPPath(dropPath);
          for (int i = 0; i < 3; i++) {
            robot.followPPPath(toTruss);
            robot.followPPPath(toBackdrop);
            robot.followPPPath(backToTruss);
                }
                robot.followPPPath(toTruss);
                robot.followPPPath(toBackdrop);
                robot.update();
            }
    robot.stop();
  }
}
