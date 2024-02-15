package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RL23 {
    boolean logi=false;
    LinearOpMode op;
    BradBot robot;
    int bark = 0;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] intake = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence drop;


    TrajectorySequence[] parky = new TrajectorySequence[3];



    public RL23(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(-36,-60.5,toRadians(-90));
        robot.roadrun.setPoseEstimate(startPose);

        spikey[0] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-47, -38, toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-44, -42, toRadians(-90)))
                .build();

        spikey[1] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-34,-36,toRadians(-90)))
                .build();

        spikey[2] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-38,-38,toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-32,-38,toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-32.5,-34,toRadians(-180)))

                .build();
        intake[0] = robot.roadrun
                .trajectorySequenceBuilder(spikey[0].end())
                .lineToLinearHeading(new Pose2d(-50,-31.25, toRadians(-180)))
                .build();
        intake[1] = robot.roadrun
                .trajectorySequenceBuilder(spikey[1].end())
                .lineToLinearHeading(new Pose2d(-51,-30.25, toRadians(-185)))
                .build();
        intake[2] = robot.roadrun
                .trajectorySequenceBuilder(spikey[0].end())
                .lineToLinearHeading(new Pose2d(-50,-31.25, toRadians(-180)))
                .build();


        if (!isLogi) {
            droppy[0] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(intake[0].end())
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(-28, -58.5), toRadians(3))
                            .splineToConstantHeading(new Vector2d(3, -60), toRadians(0))
                            .splineTo(new Vector2d(46.3, -29), toRadians(0))
                            .build();

            droppy[1] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(intake[1].end())
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(-28, -58.5), toRadians(3))
                            .splineToConstantHeading(new Vector2d(3, -60), toRadians(0))
                            .splineTo(new Vector2d(46.3, -35.25), toRadians(0))
                            .build();

            droppy[2] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(intake[2].end())
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(-28, -58.5), toRadians(3))
                            .splineToConstantHeading(new Vector2d(3, -59), toRadians(0))
                            .splineTo(new Vector2d(46.3, -41.5), toRadians(0))
                            .build();
        } else{


        }

        backToStack[0] = robot.roadrun
                .trajectorySequenceBuilder(droppy[0].end())
                .splineToConstantHeading(new Vector2d(3, -58.5), toRadians(3))
                .splineToConstantHeading(new Vector2d(-28, -58.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(-50, -35.25), toRadians(0))
                .build();
        backToStack[1] = robot.roadrun
                .trajectorySequenceBuilder(droppy[1].end())
                .setReversed(false)
                .splineTo(new Vector2d(3, -58.5), toRadians(180))
                .splineToConstantHeading(new Vector2d(-28, -58.5), toRadians(180))
                .splineToConstantHeading(new Vector2d(-53, -35.25), toRadians(150))
                .build();
        backToStack[2] = robot.roadrun
                .trajectorySequenceBuilder(droppy[2].end())
                .splineToConstantHeading(new Vector2d(10, -58.5), toRadians(-180))
                .splineToConstantHeading(new Vector2d(-30, -58.5), toRadians(-180))
                .splineToConstantHeading(new Vector2d(-50, -35.25), toRadians(-180))
                .build();
        drop = robot.roadrun.trajectorySequenceBuilder(backToStack[0].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, -58.5), toRadians(-180))
                .splineToConstantHeading(new Vector2d(10, -58.5), toRadians(-180))
                .splineToConstantHeading(new Vector2d(46.3, -41.5), toRadians(-180))
                .build();



        parky[0] = robot.roadrun
                .trajectorySequenceBuilder(droppy[0].end())
                .lineToLinearHeading(new Pose2d(43.3,-29,toRadians(-180)))
                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
                .lineToLinearHeading(new Pose2d(55,-60,toRadians(-180)))
                .build();

        parky[1] = robot.roadrun
                .trajectorySequenceBuilder(droppy[1].end())
                .lineToLinearHeading(new Pose2d(43.3,-35.5,toRadians(-180)))
                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
                .lineToLinearHeading(new Pose2d(55,-60,toRadians(-180)))
                .build();

        parky[2] = robot.roadrun
                .trajectorySequenceBuilder(droppy[2].end())
                .lineToLinearHeading(new Pose2d(43.3,-41.5,toRadians(-180)))
                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
                .lineToLinearHeading(new Pose2d(55,-60,toRadians(-180)))
                .build();

        robot.dropServo(1);
        robot.setRight(false);
        robot.setBlue(false);
        robot.observeSpike();
        robot.hoverArm();
    }
    public void waitForStart(){
        while (!op.isStarted() || op.isStopRequested()) {
            bark = robot.getSpikePos();
            op.telemetry.addData("pixel", bark);
            packet.put("spike", bark);
            robot.update();
        }
        op.resetRuntime();
    }
    public void purp()
    {
        robot.queuer.queue(false, true);
        robot.upAuto();
        robot.purpurAuto();
        robot.queuer.addDelay(1.0);
        robot.followTrajSeq(spikey[bark]);
        robot.queuer.addDelay(0.0);
        robot.dropAuto(0);
    }

    public void intake(int height){
        robot.queuer.addDelay(0.0);
        robot.followTrajSeq(intake[bark]);
        robot.resetAuto();
        robot.intakeAuto(height);
    }
    public void cycleIntake(int height){
        robot.followTrajSeq(backToStack[bark]);
        robot.intakeAuto(height);
        robot.resetAuto();
        robot.grabAuto();
    }
    public void cycleDrop(){
        robot.queuer.waitForFinish();
        robot.grabAuto();
        robot.followTrajSeq(drop);
        robot.lowAuto();
        robot.drop();
    }
    public void pre(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(droppy[bark]);
        robot.queuer.addDelay(1.5);
        robot.veryLowAuto();
        robot.drop();
    }

    public void park(){
        robot.queuer.addDelay(.5);
        robot.resetAuto();
        robot.followTrajSeq(parky[bark]);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
    }

    public void update(){
        robot.update();
    }

    public boolean isAutDone(){
        return !robot.queuer.isFullfilled()&&op.time<29.8;
    }



}
