package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RRMAX {
    boolean logi=false;
    LinearOpMode op;
    BradBot robot;
    int bark = 1;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence drop, drop2, intake, intake2, intake3, intake4, park;

    public RRMAX(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(17,-62.5,toRadians(-90));
        robot.roadrun.setPoseEstimate(startPose);


        spikey[1] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(16.5, -37.5, toRadians(-91)))
                .addTemporalMarker(robot::done)
                .build();


        if (!isLogi) {


            droppy[1] = robot.roadrun.trajectorySequenceBuilder(spikey[1].end())
                    .lineToLinearHeading(new Pose2d(48, -36.25, toRadians(-180)))
//                    .addTemporalMarker(robot::done)
                    .build();

        } else{


        }
        intake = robot.roadrun.trajectorySequenceBuilder(droppy[bark].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(17, -36.25), toRadians(180))
//                .splineToConstantHeading(new Vector2d(-30, -34.5), toRadians(180))
                .splineToConstantHeading(new Vector2d(-52.2, -34.25), toRadians(177))
                .addTemporalMarker(robot::done)
                .build();
        intake2 = robot.roadrun.trajectorySequenceBuilder(droppy[bark].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(18.5, -35.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-31.5, -34.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-52.3, -34.25), toRadians(177))
                .addTemporalMarker(robot::done)
                .build();
        intake3 = robot.roadrun.trajectorySequenceBuilder(droppy[bark].end())
                .setReversed(false)
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(42 ))
                .splineToConstantHeading(new Vector2d(18.5, -35.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-18.5, -35.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-51.7, -22.25), toRadians(170))
                .addTemporalMarker(robot::done)
                .build();
        intake4 = robot.roadrun.trajectorySequenceBuilder(droppy[bark].end())
                .setReversed(false)
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(41 ))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(90,4,15))
                .splineToConstantHeading(new Vector2d(18.5, -35.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-18.5, -35.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-53, -20.25), toRadians(170))
                .addTemporalMarker(robot::done)
                .build();
        drop = robot.roadrun.trajectorySequenceBuilder(intake.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, -35.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -38.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(49,-38.25), toRadians(0))
//                .addTemporalMarker(robot::done)
                .build();
    drop2 =
        robot
            .roadrun
            .trajectorySequenceBuilder(intake3.end())
            .setReversed(true)
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(43))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(100,5,15))
            .splineToConstantHeading(new Vector2d(-23, -36.25), toRadians(0))
            .splineToConstantHeading(new Vector2d(5, -36.25), toRadians(0))
            .splineToConstantHeading(new Vector2d(48, -36.25), toRadians(0))
//            .addTemporalMarker(robot::done)
            .build();
        park = robot.roadrun.trajectorySequenceBuilder(drop2.end())
                .lineToLinearHeading(new Pose2d(43, -36.25,toRadians(-180)))
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
        robot.followTrajSeq(spikey[bark],-40,true,false);
        robot.lowAuto(false);
    }

    public void intake(int height){
        robot.followTrajSeq(intake);
        robot.resetAuto();
        robot.queuer.addDelay(0.2);

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
    public void cycleIntake4(int height){
        robot.followTrajSeq(intake4);
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
        robot.lessLowAuto(false);
        robot.drop2();
    }
    public void pre(){
        robot.followTrajSeq(droppy[bark]);
        robot.yellowAuto(false);
        robot.drop();
    }

    public void park(){
        robot.followTrajSeq(park);
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
