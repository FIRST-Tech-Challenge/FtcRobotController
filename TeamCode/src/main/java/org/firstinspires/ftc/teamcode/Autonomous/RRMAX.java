package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RRMAX {
    boolean logi=false;
    LinearOpMode op;
    BradBot robot;
    int bark = 1;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence drop, drop2, intake, intake2, intake3, park;

    public RRMAX(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(17,-62.5,toRadians(-90));
        robot.roadrun.setPoseEstimate(startPose);


        spikey[1] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(16.5, -38.5, toRadians(-91)))
                .addTemporalMarker(robot::done)
                .build();


        if (!isLogi) {


            droppy[1] = robot.roadrun.trajectorySequenceBuilder(spikey[1].end())
                    .lineToLinearHeading(new Pose2d(48.5, -36.25, toRadians(-180)))
                    .addTemporalMarker(robot::done)
                    .build();

        } else{


        }
        intake = robot.roadrun.trajectorySequenceBuilder(droppy[bark].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(17, -35.25), toRadians(180))
//                .splineToConstantHeading(new Vector2d(-30, -34.5), toRadians(180))
                .splineToConstantHeading(new Vector2d(-52.5, -34.25), toRadians(180))
                .build();
        intake2 = robot.roadrun.trajectorySequenceBuilder(droppy[bark].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(18.5, -35.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-31.5, -34.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-53, -34.25), toRadians(180))
                .build();
        intake3 = robot.roadrun.trajectorySequenceBuilder(droppy[bark].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(18.5, -35.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-31.5, -34.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-53, -34.25), toRadians(180))
                .build();
        drop = robot.roadrun.trajectorySequenceBuilder(intake.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, -37.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -37.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(48.5,-37.25), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        drop2 = robot.roadrun.trajectorySequenceBuilder(intake2.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, -37.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -37.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(48.5,-37.25), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        park = robot.roadrun.trajectorySequenceBuilder(drop2.end())
                .lineToLinearHeading(new Pose2d(43.8,-40, toRadians(-180)))
                .lineToLinearHeading(new Pose2d(45, -60,toRadians(-180)))
                .build();

        robot.setRight(true);
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
        bark=1;
        time=0;
    }
    public void purp()
    {
        robot.queuer.queue(false, true);
        robot.upAuto();
        robot.purpurAuto();
        robot.followTrajSeq(spikey[bark]);
        robot.queuer.waitForFinish();
        robot.dropAuto(0);
    }

    public void intake(int height){
        robot.followTrajSeq(intake);
        robot.queuer.addDelay(0.2);
        robot.resetAuto();
        robot.intakeAuto(height);
    }
    public void cycleIntake(int height){
        robot.followTrajSeq(intake);
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.2);
        robot.resetAuto();
    }
    public void cycleIntake2(int height){
        robot.followTrajSeq(intake2);
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.2);
        robot.resetAuto();
    }
    public void cycleIntake3(int height){
        robot.followTrajSeq(intake3);
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.2);
        robot.resetAuto();
    }
    public void cycleDrop(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(drop);
        robot.grabAuto();
        robot.lowAuto(false);
        robot.drop();
    }
    public void cycleDrop2(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(drop2);
        robot.grabAuto();
        robot.lowAuto(false);
        robot.drop();
    }
    public void pre(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(droppy[bark]);
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
