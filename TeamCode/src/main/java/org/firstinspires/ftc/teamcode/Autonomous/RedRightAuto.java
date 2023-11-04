package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
@Autonomous(name = "RedRightPreParkAuto")
public class RedRightAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        robot.roadrun.setPoseEstimate(new Pose2d(15.5, -56, Math.toRadians(-90)));
        int pos = 0;
        TrajectorySequence[] preload = new TrajectorySequence[3];
        preload[0] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(15.5, -56, Math.toRadians(-90)))
                .setReversed(true) //spike1
                .lineToLinearHeading(new Pose2d(16, -35+2, toRadians(0)))
                .splineTo(new Vector2d(8,-35 + 2), toRadians(-180))
                .setReversed(false)
                .addTemporalMarker(robot::done)
                .build();
        preload[1] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(15.5, -63.25, Math.toRadians(-90)))
                .setReversed(true) //spike2
                .lineToLinearHeading(new Pose2d(12.5, -33-2, toRadians(-90)))
                .lineToLinearHeading(new Pose2d(12.5, -58, toRadians(-180)))
                .addTemporalMarker(robot::done)
                .build();
        preload[2] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(15.5, -63.25, Math.toRadians(-90)))
                .setReversed(true) //spike3
                .lineToLinearHeading(new Pose2d(35,-31,toRadians(0)))
                .setReversed(false)
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence[] dropAndPark = new TrajectorySequence[3];
        dropAndPark[0]= robot.roadrun.trajectorySequenceBuilder(preload[0].end())
                .splineTo(new Vector2d(51 + 4, -30.5 + 2), toRadians(0))
                .lineToLinearHeading(new Pose2d(51 + 4, -60, toRadians(0)))
                .lineToLinearHeading(new Pose2d(60 + 4, -60, toRadians(0)))
                .addTemporalMarker(robot::done)
                .build();
        dropAndPark[1]= robot.roadrun.trajectorySequenceBuilder(preload[1].end())
                .splineTo(new Vector2d(51 + 4, -35.5 + 2), toRadians(0))
                .lineToLinearHeading(new Pose2d(51 + 4, -60, toRadians(180)))
                .lineToLinearHeading(new Pose2d(60 + 4, -60, toRadians(180)))
                .addTemporalMarker(robot::done)
                .build();
        dropAndPark[2] = robot.roadrun.trajectorySequenceBuilder(preload[2].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(51+4, -41.5 + 2,toRadians(0)))
                .lineToLinearHeading(new Pose2d(51+4, -60, toRadians(0)))
                .lineToLinearHeading(new Pose2d(60+4, -60, toRadians(0)))
                .addTemporalMarker(robot::done)
                .build();
        while(!isStarted()){
//            pos=0;
        } //willy phone password 272714
        while(!isStopRequested()&&opModeIsActive()&&!robot.queuer.isFullfilled()){
            robot.followTrajSeq(preload[pos]);
            robot.preloadAuto();
            robot.followTrajSeq(dropAndPark[pos]);
            robot.queuer.setFirstLoop(false);
            robot.update();
        }
    }
}
