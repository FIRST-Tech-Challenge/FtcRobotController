package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
@Config
@Autonomous(name = "RedLeftPreParkAUto")
public class RedLeftAuto extends LinearOpMode {
    public static int pos = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        robot.roadrun.setPoseEstimate(new Pose2d(-38.5,-56, Math.toRadians(-90)));
        TrajectorySequence[] spikePosition = new TrajectorySequence[3];

        spikePosition[0] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -56, Math.toRadians(-90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-49, -37, toRadians(-90)))
                .addTemporalMarker(robot::done)
                .build();
        spikePosition[1] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -56, Math.toRadians(-90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-35.5, -30, toRadians(-90)))
                .addTemporalMarker(robot::done)
                .build();
        spikePosition[2] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -56, Math.toRadians(-90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-38.5, -34, toRadians(180)))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence[] throughTruss = new TrajectorySequence[3];
        throughTruss[0] = robot.roadrun.trajectorySequenceBuilder(spikePosition[0].end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-46, -59, toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(10, -56), toRadians(0))
                .splineTo(new Vector2d(56, -30.5), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        throughTruss[1] = robot.roadrun.trajectorySequenceBuilder(spikePosition[1].end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-46, -59, toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(10, -56), toRadians(0))
                .splineTo(new Vector2d(56, -35.5), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        throughTruss[2] = robot.roadrun.trajectorySequenceBuilder(spikePosition[2].end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-46, -59, toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(10, -56), toRadians(0))
                .splineTo(new Vector2d(56, -41.5), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence[] dropAndPark = new TrajectorySequence[3];
        dropAndPark[0]= robot.roadrun.trajectorySequenceBuilder(throughTruss[0].end())
                .lineToLinearHeading(new Pose2d(56, -60, toRadians(180)))
                .lineToLinearHeading(new Pose2d(60, -60, toRadians(180)))
                .addTemporalMarker(robot::done)
                .build();
        dropAndPark[1]= robot.roadrun.trajectorySequenceBuilder(throughTruss[1].end())
                .lineToLinearHeading(new Pose2d(56, -60, toRadians(180)))
                .lineToLinearHeading(new Pose2d(60, -60, toRadians(180)))
                .addTemporalMarker(robot::done)
                .build();
        dropAndPark[2] = robot.roadrun.trajectorySequenceBuilder(throughTruss[2].end())
                .lineToLinearHeading(new Pose2d(56, -60, toRadians(180)))
                .lineToLinearHeading(new Pose2d(60, -60, toRadians(180)))
                .addTemporalMarker(robot::done)
                .build();
        while(!isStarted()){
//            pos=0;
        }
        while(!isStopRequested()&&opModeIsActive()&&!robot.queuer.isFullfilled()){
            robot.followTrajSeq(spikePosition[pos]);
            robot.queuer.waitForFinish();
            robot.preloadAuto();
            robot.queuer.waitForFinish();
            robot.followTrajSeq(throughTruss[pos]);
            robot.followTrajSeq(dropAndPark[pos]);
            robot.queuer.setFirstLoop(false);
            robot.update();
        }
    }
}
