package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class RLRR extends LinearOpMode {
    int bark = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        Pose2d startPose = new Pose2d(-38,-61,toRadians(-90));
        robot.roadrun.setPoseEstimate(startPose);
        TrajectorySequence[] spikey = new TrajectorySequence[3];

        spikey[0] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-46, -38, toRadians(-90)))
                .build();

        spikey[1] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-43,-36,toRadians(-90)))
                .build();

        spikey[2] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,-38,toRadians(-90)))
                .build();
//
//        spikey[2] = robot.roadrun
//                .trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(0,0,toRadians(0)))
//                .build();

        TrajectorySequence[] pathy = new TrajectorySequence[3];

        pathy[0] = robot.roadrun
                .trajectorySequenceBuilder(spikey[0].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-46,-57, toRadians(-180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(20,-57, toRadians(-180)))
                .build();

//        pathy[1] = robot.roadrun
//                .trajectorySequenceBuilder(spikey[1].end())
//                .lineToLinearHeading(new Pose2d(0,0,toRadians(0)))
//                .build();
//
//        pathy[2] = robot.roadrun
//                .trajectorySequenceBuilder(spikey[1].end())
//                .lineToLinearHeading(new Pose2d(0,0,toRadians(0)))
//                .build();

        TrajectorySequence[] droppy = new TrajectorySequence[3];

        droppy[0] = robot.roadrun
                .trajectorySequenceBuilder(pathy[0].end())
                .lineToLinearHeading(new Pose2d(47.2,-29,toRadians(-180)))
                .build();

//        droppy[1] = robot.roadrun
//                .trajectorySequenceBuilder(pathy[1].end())
//                .lineToLinearHeading(new Pose2d(0,0,toRadians(0)))
//                .build();
//
//        droppy[2] = robot.roadrun
//                .trajectorySequenceBuilder(pathy[2].end())
//                .lineToLinearHeading(new Pose2d(0,0,toRadians(0)))
//                .build();

        TrajectorySequence[] parky = new TrajectorySequence[3];

        parky[0] = robot.roadrun
                .trajectorySequenceBuilder(droppy[0].end())
                .lineToLinearHeading(new Pose2d(50,-10,toRadians(-180)))
                .build();

//        parky[1] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[1].end())
//                .lineToLinearHeading(new Pose2d(0,0,toRadians(0)))
//                .build();
//
//        parky[2] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[2].end())
//                .lineToLinearHeading(new Pose2d(0,0,toRadians(0)))
//                .build();
        robot.dropServo(1);
        robot.setRight(false);
        robot.setBlue(false);
        robot.observeSpike();

        while (!isStarted() || isStopRequested()) {
            bark = robot.getSpikePos();
            telemetry.addData("pixel", bark);
            packet.put("spike", bark);
            robot.update();
        }
        while (!isStopRequested() && opModeIsActive()) {
            robot.queuer.queue(false, true);
            robot.upAuto();
            robot.purpurAuto();
            robot.queuer.addDelay(1.0);
            robot.followTrajSeq(spikey[0]);
            robot.queuer.addDelay(0.2);
            robot.dropAuto(0);
            robot.queuer.addDelay(0.3);
            robot.followTrajSeq(pathy[0]);
            robot.queuer.addDelay(.6);
            robot.resetAuto();
            robot.grabSupAuto();
            robot.followTrajSeq(droppy[0]);
            robot.queuer.addDelay(.5);
            robot.lowAuto();
            robot.drop();
            robot.queuer.addDelay(.5);
            robot.resetAuto();
            robot.followTrajSeq(parky[0]);
            robot.queuer.waitForFinish();
            robot.queuer.queue(false, true);
            robot.update();
        }
    }
}
