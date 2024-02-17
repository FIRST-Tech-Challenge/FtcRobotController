package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedLeftUltra2+0")
@Config
public class RLRRU extends LinearOpMode {
    int bark = 1;
    boolean previousCheck;


    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        Pose2d startPose = new Pose2d(-38,-60,toRadians(-90));
        robot.roadrun.setPoseEstimate(startPose);
        TrajectorySequence[] spikey = new TrajectorySequence[3];

        spikey[0] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-46, -38, toRadians(-90)))
                .build();

        spikey[1] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-36,-36,toRadians(-90)))
                .build();

        spikey[2] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-40,-38,toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-36,-35,toRadians(-180)))
                .build();

        TrajectorySequence[] pathy = new TrajectorySequence[3];

        pathy[0] = robot.roadrun
                .trajectorySequenceBuilder(spikey[0].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-40,-58.5, toRadians(-180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(20,-58.5, toRadians(-145)))
                .build();

        pathy[1] = robot.roadrun
                .trajectorySequenceBuilder(spikey[1].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-46,-58.5, toRadians(-180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(20,-58.5, toRadians(-145)))
                .build();

        pathy[2] = robot.roadrun
                .trajectorySequenceBuilder(spikey[1].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-46,-58.5, toRadians(-180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(20,-58.5, toRadians(-145)))
                .build();

        TrajectorySequence[] detecty = new TrajectorySequence[3];

        detecty[0] = robot.roadrun
                .trajectorySequenceBuilder(pathy[0].end())
                .lineToLinearHeading(new Pose2d(21,-58.5, toRadians(-180)))
                .build();

        detecty[1] = robot.roadrun
                .trajectorySequenceBuilder(pathy[1].end())
                .lineToLinearHeading(new Pose2d(21,-58.5, toRadians(-180)))
                .build();

        detecty[2] = robot.roadrun
                .trajectorySequenceBuilder(pathy[1].end())
                .lineToLinearHeading(new Pose2d(21,-58.5, toRadians(-180)))
                .build();

        TrajectorySequence[] droppy = new TrajectorySequence[3];

        droppy[0] = robot.roadrun
                .trajectorySequenceBuilder(detecty[0].end())
                .lineToLinearHeading(new Pose2d(46.4,-29.5,toRadians(-180)))
                .build();

        droppy[1] = robot.roadrun
                .trajectorySequenceBuilder(detecty[1].end())
                .lineToLinearHeading(new Pose2d(40.5,-35,toRadians(-180)))
                .lineToLinearHeading(new Pose2d(46.4,-35.5,toRadians(-180)))
                .build();

        droppy[2] = robot.roadrun
                .trajectorySequenceBuilder(detecty[2].end())
                .lineToLinearHeading(new Pose2d(40.5,-35,toRadians(-180)))
                .lineToLinearHeading(new Pose2d(46.4,-41.5,toRadians(-180)))
                .build();



        TrajectorySequence[] parky = new TrajectorySequence[3];

        parky[0] = robot.roadrun
                .trajectorySequenceBuilder(droppy[0].end())
                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
                .build();

        parky[1] = robot.roadrun
                .trajectorySequenceBuilder(droppy[1].end())
                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
                .build();

        parky[2] = robot.roadrun
                .trajectorySequenceBuilder(droppy[2].end())
                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
                .build();

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
            previousCheck = robot.checkAlliance();
            op.telemetry.addData("check", previousCheck);
            robot.queuer.queue(false, true);
//            robot.upAuto();
//            robot.purpurAuto();
            robot.queuer.addDelay(1.0);
            robot.followTrajSeq(spikey[bark]);
            robot.queuer.addDelay(0.2);
            robot.dropAuto(0);
            robot.queuer.addDelay(0.3);
            robot.followTrajSeq(pathy[bark]);
            robot.queuer.addDelay(.6);
//            robot.resetAuto();
//            robot.grabSupAuto();
            robot.followTrajSeq(detecty[bark], !previousCheck);
            robot.followTrajSeq(droppy[bark]);
            robot.queuer.addDelay(.5);
//            robot.lowAuto();
            robot.queuer.addDelay(1.5);
//            robot.veryLowAuto();
//            robot.drop();
            robot.queuer.addDelay(.5);
//            robot.resetAuto();
            robot.followTrajSeq(parky[bark]);
            robot.queuer.waitForFinish();
            robot.queuer.queue(false, true);
            robot.update();
        }
    }
}
