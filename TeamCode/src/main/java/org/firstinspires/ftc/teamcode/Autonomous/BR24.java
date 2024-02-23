package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BR24 {
    boolean logi=false;
    LinearOpMode op;
    BradBot robot;
    int bark = 0;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence drop, intake, park;

    public BR24(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(17, 61, Math.toRadians(90));
        Vector2d[] droppos = new Vector2d[3];
        droppos[0] = new Vector2d(46.4, 29);
        droppos[1] = new Vector2d(46.4, 35.25);
        droppos[2] = new Vector2d(46.4, 41.5);
        robot.roadrun.setPoseEstimate(startPose);

        spikey[0] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(17,61, toRadians(90)))
                .lineToLinearHeading(new Pose2d(24.5,45, toRadians(90))).build();

        spikey[1] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(17,61,toRadians(90)))
                .lineToLinearHeading(new Pose2d(16.5, 36.5, toRadians(90))).build();

        spikey[2] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(17,61, toRadians(90)))
                .lineToLinearHeading(new Pose2d(16.5, 43, toRadians(60)))
                .lineToLinearHeading(new Pose2d(9.5, 40, toRadians(50))).build();

        if (!isLogi) {
            droppy[0] = robot.roadrun.trajectorySequenceBuilder(spikey[0].end())
                    .lineToLinearHeading(new Pose2d(46.4, 29, toRadians(180))).build();

            droppy[1] = robot.roadrun.trajectorySequenceBuilder(spikey[1].end())
                    .lineToLinearHeading(new Pose2d(46.4, 35.25, toRadians(180))).build();

            droppy[2] = robot.roadrun.trajectorySequenceBuilder(spikey[2].end())
                    .lineToLinearHeading(new Pose2d(46.4, 41.5, toRadians(180))).build();
        } else{


        }
        intake = robot.roadrun.trajectorySequenceBuilder(droppy[bark].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(17, 3), toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, 3), toRadians(180))
                .splineToConstantHeading(new Vector2d(-52, 3), toRadians(180))
                .build();
        drop = robot.roadrun.trajectorySequenceBuilder(intake.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, 4), toRadians(0))
                .splineToConstantHeading(new Vector2d(5, 4), toRadians(0))
                .splineToConstantHeading(droppos[bark], toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        park = robot.roadrun.trajectorySequenceBuilder(drop.end())
                .lineToLinearHeading(new Pose2d(43.8,40, toRadians(180)))
                .lineToLinearHeading(new Pose2d(45, 60,toRadians(180)))
                .build();

        robot.setRight(true);
        robot.setBlue(true);
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
        time=0;
    }
    public void purp()
    {
        robot.queuer.queue(false, true);
        robot.upAuto();
        robot.purpurAuto();
        robot.queuer.waitForFinish();
        robot.followTrajSeq(spikey[bark]);
        robot.dropAuto(0);
    }

    public void intake(int height){
        robot.queuer.addDelay(0.0);
        robot.followTrajSeq(intake);
        robot.resetAuto();
        robot.intakeAuto(height);
    }
    public void cycleIntake(int height){
        robot.queuer.addDelay(0.2);
        robot.followTrajSeq(intake);
        robot.intakeAuto(height);
        robot.resetAuto();
    }
    public void cycleDrop(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(drop);
        robot.grabAuto();
        robot.lowAuto(false);
        robot.drop();
    }
    public void pre(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(droppy[bark]);
        robot.grabAuto();
        robot.lowAuto(false);
        robot.yellowAuto(false);
        robot.drop();
    }

    public void park(){
        robot.queuer.addDelay(.5);
        robot.resetAuto();
        robot.followTrajSeq(park);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
    }

    public void update(){
        robot.update();
        robot.queuer.setFirstLoop(false);
    }

    public boolean isAutDone(){
        return !robot.queuer.isFullfilled()&&op.time<29.8;
    }

}
