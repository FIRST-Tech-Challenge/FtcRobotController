package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.imuMultiply;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RR24 {
    boolean logi=false;
    LinearOpMode op;
    BradBot robot;
    int bark = 1;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence drop, drop2, drop3, intake, intake2, intake3, park;

    public RR24(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(17,-64.5,toRadians(-90));
        robot.roadrun.setPoseEstimate(startPose);
        imuMultiply = 1.041 + .005*(robot.getVoltage()-12.5);

        spikey[0] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(11.5, -37, toRadians(0)), toRadians(180))
                .build();

        spikey[1] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(16.5, -34.5, toRadians(-91)))
                .lineToLinearHeading(new Pose2d(16.5, -40.5, toRadians(-91)))
                .addTemporalMarker(robot :: done)
                .build();

        spikey[2] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(21.5,-45, toRadians(-90)))
                .lineToLinearHeading(new Pose2d(21.5,-53.5, toRadians(-90)))
                .addTemporalMarker(robot :: done)
                .build();
        if (!isLogi) {
            droppy[0] = robot.roadrun.trajectorySequenceBuilder(spikey[0].end())
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(46.4, -33, toRadians(-179)),toRadians(0))
                    .build();

            droppy[1] = robot.roadrun.trajectorySequenceBuilder(spikey[1].end())
                    .lineToLinearHeading(new Pose2d(45, -35.25, toRadians(-180))).build();

            droppy[2] = robot.roadrun.trajectorySequenceBuilder(spikey[2].end())
                    .lineToLinearHeading(new Pose2d(46.4, -41.5, toRadians(-180))).build();
        } else{


        }
        intake = robot.roadrun.trajectorySequenceBuilder(droppy[bark].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, -11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35,5,14))
                .splineToConstantHeading(new Vector2d(7, -11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, -12.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-55.25, -13.25), toRadians(180))
                .build();

        drop = robot.roadrun.trajectorySequenceBuilder(intake.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, -11.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -11.25), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                .splineToConstantHeading(new Vector2d(43, -33), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(55))

                .build();
        intake2 = robot.roadrun.trajectorySequenceBuilder(drop.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, -11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35,5,14))
                .splineToConstantHeading(new Vector2d(7, -11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, -11.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-54, -11.25), toRadians(180))
                .build();
        drop2 = robot.roadrun.trajectorySequenceBuilder(intake2.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, -11.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -11.25), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(43, -33), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(55))
                .build();
        intake3 = robot.roadrun.trajectorySequenceBuilder(drop2.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, -11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35,5,14))
                .splineToConstantHeading(new Vector2d(7, -11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(0, -11.25), toRadians(180))
                .splineToSplineHeading(new Pose2d(-56, -17.75, toRadians(210)), toRadians(210))
                .build();
        drop3 = robot.roadrun.trajectorySequenceBuilder(intake3.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-30, -11.25, toRadians(-180)), toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -11.25), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(43, -33), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(55))
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
        time=0;
    }
    public void purp()
    {
        bark=2;
        robot.queuer.queue(false, true);
        robot.queuer.waitForFinish();
        robot.followTrajSeq(spikey[bark]);
    }

    public void intake(int height){
        robot.queuer.addDelay(0.0);
        robot.followTrajSeq(intake);
//        robot.resetAuto();
//        robot.intakeAuto(height);
    }
    public void cycleIntake(int height){
        robot.followTrajSeq(intake);
        robot.intakeAuto(height);
        robot.resetAuto();
    }
    public void cycleIntake2(int height){
        robot.queuer.addDelay(0.2);
        robot.followTrajSeq(intake2);
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.3);
        robot.resetAuto();
    }
    public void cycleIntake3(int height){
        robot.queuer.addDelay(0.2);
        robot.followTrajSeq(intake3);
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.3);
        robot.resetAuto();
    }
    public void cycleDrop(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(drop);
        robot.queuer.addDelay(0.2);
        robot.grabAuto();
        robot.lowAuto(false);
        robot.drop(42.5);
    }
    public void cycleDrop2(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(drop2);
        robot.queuer.addDelay(0.2);
        robot.grabAuto();
        robot.lowAuto(false);
        robot.drop(42.5);
    }
    public void cycleDrop3(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(drop3);
        robot.queuer.addDelay(0.2);
        robot.grabAuto();
        robot.lowAuto(false);
        robot.drop(42.5);
    }
    public void pre(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(droppy[bark]);
        robot.lowAuto(false);
        robot.yellowAuto(false);
        if(bark==0)
            robot.drop(46);
        else if(bark==1)
            robot.drop(45);
        else
            robot.drop(46);
    }

    public void park(){

        robot.followTrajSeq(park);
        robot.queuer.addDelay(.5);
        robot.resetAuto();
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
