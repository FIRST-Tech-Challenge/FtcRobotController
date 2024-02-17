package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Autonomous(name = "logiBlueRight2+0")
@Config
public class logiBRRR extends LinearOpMode {
//    int bark = 1;


    @Override
    public void runOpMode() throws InterruptedException {
//        BradBot robot = new BradBot(this, false);
//        robot.roadrun.setPoseEstimate(new Pose2d(17, 61, Math.toRadians(90)));
//
//        TrajectorySequence[] preload = new TrajectorySequence[3];
//        preload[2] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(17,61, toRadians(90)))
//                .lineToLinearHeading(new Pose2d(16.5, 43, toRadians(60)))
//                .lineToLinearHeading(new Pose2d(9.5, 40, toRadians(50))).build();
//        preload[1] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(17,61,toRadians(90)))
//                .lineToLinearHeading(new Pose2d(16.5, 36.5, toRadians(91))).build();
//        preload[0] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(17,61, toRadians(90)))
//                .lineToLinearHeading(new Pose2d(24.5,43, toRadians(90))).build();
//        TrajectorySequence[] preToStack = new TrajectorySequence[3];
//        preToStack[2] = robot.roadrun.trajectorySequenceBuilder(preload[2].end())
//                .lineToLinearHeading(new Pose2d(36.4, 35, toRadians(180)))
//                .lineToLinearHeading(new Pose2d(46.5, 29.5, toRadians(180))).build();
//        preToStack[1] = robot.roadrun.trajectorySequenceBuilder(preload[1].end())
//                .lineToLinearHeading(new Pose2d(16.5, 39.5, toRadians(91)))
//                .lineToLinearHeading(new Pose2d(36.4, 37, toRadians(180)))
//                .lineToLinearHeading(new Pose2d(46.5, 35.5, toRadians(180))).build();
//        preToStack[0] = robot.roadrun.trajectorySequenceBuilder(preload[0].end())
////                .lineToLinearHeading(new Pose2d(36.4, 35, toRadians(180)))
//                .lineToLinearHeading(new Pose2d(46.5, 41.5, toRadians(180))).build();
//        TrajectorySequence[] park = new TrajectorySequence[3];
//        park[0] = robot.roadrun.trajectorySequenceBuilder(preToStack[0].end())
//                .lineToLinearHeading(new Pose2d(43, 29, toRadians(180)))
//                .lineToLinearHeading(new Pose2d(43, 58, toRadians(180)))
//                .lineToLinearHeading(new Pose2d(58, 58, toRadians(180)))
//                .build();
//        park[1] = robot.roadrun.trajectorySequenceBuilder(preToStack[1].end())
//                .lineToLinearHeading(new Pose2d(43, 35.5, toRadians(180)))
//                .lineToLinearHeading(new Pose2d(43, 58, toRadians(180)))
//                .lineToLinearHeading(new Pose2d(58, 58, toRadians(180)))
//                .build();
//        park[2] = robot.roadrun.trajectorySequenceBuilder(preToStack[2].end())
//                .lineToLinearHeading(new Pose2d(43, 41.5, toRadians(180)))
//                .lineToLinearHeading(new Pose2d(43, 58, toRadians(180)))
//                .lineToLinearHeading(new Pose2d(58, 58, toRadians(180)))
//                .build();
//
//        robot.setRight(false);
//        robot.setBlue(false);
//        robot.observeSpike();
        BR20 aut = new BR20(this, true);
//        while (!isStarted() && !isStopRequested()) {
//            bark = robot.getSpikePos();
//            telemetry.addData("pixel", bark);
//            packet.put("pix", bark);
//            robot.update();
//        }
//    bark=0;
        aut.waitForStart();
        while (!isStopRequested() && opModeIsActive()&& aut.isAutDone()) {
            aut.purp();
            aut.pre();
            aut.park();
            aut.update();
        }
    }
}
