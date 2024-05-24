package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedLeft2+3")
@Config
public class RedLeft23 extends LinearOpMode {
//    int bark = 1;

    @Override
    public void runOpMode() throws InterruptedException {
//        BradBot robot = new BradBot(this, false);
//        Pose2d startPose = new Pose2d(-36,-60,toRadians(-90));
//        robot.roadrun.setPoseEstimate(startPose);
//        TrajectorySequence[] spikey = new TrajectorySequence[3];
//
//        spikey[0] = robot.roadrun
//                .trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(-47, -38, toRadians(-90)))
//                .lineToLinearHeading(new Pose2d(-44, -42, toRadians(-90)))
//                .build();
//
//        spikey[1] = robot.roadrun
//                .trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(-34,-36,toRadians(-90)))
//                .build();
//
//        spikey[2] = robot.roadrun
//                .trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(-38,-38,toRadians(-90)))
//                .lineToLinearHeading(new Pose2d(-32,-38,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(-32.5,-34,toRadians(-180)))
//
//                .build();
//
//        TrajectorySequence[] pathy = new TrajectorySequence[3];
//
//        pathy[0] = robot.roadrun
//                .trajectorySequenceBuilder(spikey[0].end())
//                .setReversed(false)
//                .lineToLinearHeading(new Pose2d(-38,-58.5, toRadians(-180)))
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(20,-58.5, toRadians(-180)))
//                .build();
//
//        pathy[1] = robot.roadrun
//                .trajectorySequenceBuilder(spikey[1].end())
//                .setReversed(false)
//                .lineToLinearHeading(new Pose2d(-38,-58.5, toRadians(-180)))
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(20,-58.5, toRadians(-180)))
//                .build();
//
//        pathy[2] = robot.roadrun
//                .trajectorySequenceBuilder(spikey[1].end())
//                .setReversed(false)
//                .lineToLinearHeading(new Pose2d(-38,-58.5, toRadians(-180)))
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(20,-58.5, toRadians(-180)))
//                .build();
//
//        TrajectorySequence[] droppy = new TrajectorySequence[3];
//
//        droppy[0] = robot.roadrun
//                .trajectorySequenceBuilder(pathy[0].end())
//                .lineToLinearHeading(new Pose2d(40.5,-32,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(46.3,-29,toRadians(-180)))
//                .build();
//
//        droppy[1] = robot.roadrun
//                .trajectorySequenceBuilder(pathy[1].end())
//                .lineToLinearHeading(new Pose2d(40.5,-35,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(46.3,-35.5,toRadians(-180)))
//                .build();
//
//        droppy[2] = robot.roadrun
//                .trajectorySequenceBuilder(pathy[2].end())
//                .lineToLinearHeading(new Pose2d(40.5,-35,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(46.3,-41.5,toRadians(-180)))
//                .build();
//
//
//
//        TrajectorySequence[] parky = new TrajectorySequence[3];
//
//        parky[0] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[0].end())
//                .lineToLinearHeading(new Pose2d(43.3,-29,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
//                .build();
//
//        parky[1] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[1].end())
//                .lineToLinearHeading(new Pose2d(43.3,-35.5,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
//                .build();
//
//        parky[2] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[2].end())
//                .lineToLinearHeading(new Pose2d(43.3,-41.5,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
//                .build();
//
//        robot.dropServo(1);
//        robot.setRight(false);
//        robot.setBlue(false);
//        robot.observeSpike();
        RL23 aut = new RL23(this, false);

        aut.waitForStart();
        while (!isStopRequested() && opModeIsActive()&& aut.isAutDone()) {
            aut.purp();
            aut.intake(5);
            aut.pre();
            aut.cycleIntake(4);
            aut.cycleDrop();
            aut.cycleIntake(2);
            aut.cycleDrop();
//            aut.park();
            aut.update();
        }
    }
}
