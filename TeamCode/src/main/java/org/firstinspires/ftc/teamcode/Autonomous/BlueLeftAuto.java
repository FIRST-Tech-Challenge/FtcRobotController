package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
@Autonomous(name = "BlueLeftPreParkAuto")
public class BlueLeftAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        robot.roadrun.setPoseEstimate(new Pose2d(-38.5,63.25, Math.toRadians(90)));
        int pos = 0;
        TrajectorySequence[] spikePosition = new TrajectorySequence[3];
        spikePosition[0] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, 63.25, Math.toRadians(-90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-46, 37, toRadians(-90)))
                .addTemporalMarker(robot::done)
                .build();
        spikePosition[1] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, 63.5, Math.toRadians(-90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-35.5, 33, toRadians(-90)))
                .addTemporalMarker(robot::done)
                .build();
        spikePosition[2] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, 63.5, Math.toRadians(-90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-38.5, 34, toRadians(-90)))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence throughTruss = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-46,37, toRadians(90)))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-46, 58, toRadians(0)))
                .splineTo(new Vector2d(10, 58), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence[] dropAndPark = new TrajectorySequence[3];
        dropAndPark[0]= robot.roadrun.trajectorySequenceBuilder(new Pose2d(10,58, toRadians(0)))
                .splineTo(new Vector2d(51, 30.5), toRadians(0))
                .lineToLinearHeading(new Pose2d(51, 60, toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 60, toRadians(0)))
                .addTemporalMarker(robot::done)
                .build();
        dropAndPark[1]= robot.roadrun.trajectorySequenceBuilder(new Pose2d(10,58,toRadians(0)))
                .splineTo(new Vector2d(51,35.5), toRadians(0))
                .lineToLinearHeading(new Pose2d(51, 60, toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 60, toRadians(0)))
                .addTemporalMarker(robot::done)
                .build();
        dropAndPark[2] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(10,58,toRadians(0)))
                .splineTo(new Vector2d(51, 41.5), toRadians(0))
                .lineToLinearHeading(new Pose2d(51, 60, toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 60, toRadians(0)))
                .addTemporalMarker(robot::done)
                .build();
        waitForStart();
        while(!isStopRequested()&&opModeIsActive()&&!robot.queuer.isFullfilled()){
            robot.followTrajSeq(spikePosition[pos]);
            robot.followTrajSeq(throughTruss);
            robot.followTrajSeq(dropAndPark[pos]);
            robot.queuer.setFirstLoop(false);
            robot.update();
        }
    }
}
