package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.funnyIMUOffset;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.imuMultiply;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFAprilCam;
import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BR24 {
    boolean logi=false, isRight,ultras = true, check = true, everChecked = false, intakey = false ;
    LinearOpMode op;
    BradBot robot;
    int bark = 0, delaySec =0;
    boolean lastCycle = false;
    boolean joever = false;
    boolean parky = false;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] intake = new TrajectorySequence[3];
    TrajectorySequence[] intake2 = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence[] drop = new TrajectorySequence[3];
    TrajectorySequence[] park= new TrajectorySequence[3], parkLeft= new TrajectorySequence[3];





    public BR24(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(15,61.5,toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);
        imuMultiply = 1.0132;
        LATERAL_MULTIPLIER = 1.4;
        lastCycle = false;
        joever = false;
        parky = false;

        spikey[0] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(9.5, 37, toRadians(0)), toRadians(180))
                .build();

        spikey[1] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12.5, 32.5, toRadians(91)))
                .lineToLinearHeading(new Pose2d(12.5, 40.5, toRadians(-1)))
                .build();

        spikey[2] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(19.5,42, toRadians(90)))
                .lineToLinearHeading(new Pose2d(19.5,54, toRadians(90)))
                .build();
        if (!isLogi) {
            droppy[0] = robot.roadrun.trajectorySequenceBuilder(spikey[0].end())
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(40, 35, toRadians(179.9)))
                    .addTemporalMarker(robot::stopAllMotors)
                    .addTemporalMarker(robot::startIMU)
                    .addTemporalMarker(()->robot.roadrun.changeIMUInterval())
                    .lineToLinearHeading(new Pose2d(46.5, 31, toRadians(179.9)))
                    .addTemporalMarker(()->robot.dropServo(0))
                    .addTemporalMarker(()->robot.dropServo(1))

                    .build();

            droppy[1] = robot.roadrun.trajectorySequenceBuilder(spikey[1].end())
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(38, 35, toRadians(179.9)))
                    .addTemporalMarker(robot::stopAllMotors)
                    .addTemporalMarker(robot::startIMU)
                    .addTemporalMarker(()->robot.roadrun.changeIMUInterval())
                    .lineToLinearHeading(new Pose2d(46, 37, toRadians(179.9)))
                    .addTemporalMarker(()->robot.dropServo(0))
                    .addTemporalMarker(()->robot.dropServo(1))
                    .build();

            droppy[2] = robot.roadrun.trajectorySequenceBuilder(spikey[2].end())
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(38, 36, toRadians(179.9)))
                    .addTemporalMarker(robot::stopAllMotors)
                    .addTemporalMarker(robot::startIMU)
                    .addTemporalMarker(()->robot.roadrun.changeIMUInterval())
                    .lineToLinearHeading(new Pose2d(46, 39, toRadians(179.9)))
                    .addTemporalMarker(()->robot.dropServo(0))
                    .addTemporalMarker(()->robot.dropServo(1))
                    .build();
        } else{


        }
        intake[0] = robot.roadrun.trajectorySequenceBuilder(droppy[0].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, 11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35,5,14))
                .splineToConstantHeading(new Vector2d(7, 11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, 11.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-56.6, 12.75), toRadians(180))
                .build();
        intake[1] = robot.roadrun.trajectorySequenceBuilder(droppy[1].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, 11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35,5,14))
                .splineToConstantHeading(new Vector2d(7, 11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, 11.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-56.6, 12.75), toRadians(180))
                .build();
        intake[2] = robot.roadrun.trajectorySequenceBuilder(droppy[2].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, 11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(34,5,14))
                .splineToConstantHeading(new Vector2d(7, 11.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, 11.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-56.6, 11.75), toRadians(180))
                .build();

        drop[0] = robot.roadrun.trajectorySequenceBuilder(intake[0].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, 13.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(20, 13.25), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(27))
                .splineToConstantHeading(new Vector2d(45.8, 35), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(43))
                .build();
        drop[1] = robot.roadrun.trajectorySequenceBuilder(intake[0].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, 11.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(20, 11.25), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(27))
                .splineToConstantHeading(new Vector2d(45.8, 35), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(43))
                .build();
        intake2[0] = robot.roadrun.trajectorySequenceBuilder(drop[0].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, 9.85), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25,5,14))
                .splineToConstantHeading(new Vector2d(7, 9.85), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, 10.5), toRadians(180))
                .splineToConstantHeading(new Vector2d(-56.5, 10.8), toRadians(180))
                .build();
        intake2[1] = robot.roadrun.trajectorySequenceBuilder(drop[1].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, 12.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30,5,14))
                .splineToConstantHeading(new Vector2d(7, 12.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, 12.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-57.2, 12.25), toRadians(180))
                .build();

        park[0] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(43,30, toRadians(180)))
                .lineToLinearHeading(new Pose2d(43.8,40, toRadians(-180)))
                .lineToLinearHeading(new Pose2d(45, 18,toRadians(-180)))
                .build();


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
            op.telemetry.addData("delaySec", delaySec);
            op.telemetry.addData("isRight", isRight);
            if (gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "addSecs")) {
                delaySec++;
            }
            if (gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "minusSecs")) {
                delaySec = min(0, delaySec - 1);
            }
            if (gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "parkRight")) {
                isRight = true;
            }
            if (gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "parkLeft")) {
                isRight = false;
            }
            robot.update();
        }
        op.resetRuntime();

//        bark=2;
        time=0;
    }
    public void purp()
    {
//        bark=2;
        if(bark==0){
            funnyIMUOffset = 3;
        }
        if(bark==1){
            funnyIMUOffset = 1.0;
        }
        if(bark ==2){
            funnyIMUOffset = -0.5;
        }
        robot.queuer.queue(false, true);
        robot.followTrajSeq(spikey[bark]);
        robot.queuer.addDelay(0.0);
    }

    public void intake(int height){
        robot.followTrajSeq(intake[bark]);
        robot.resetAuto();
        if (bark == 0) {
            robot.queuer.addDelay(1.5);
        }
        robot.intakeAuto(height);
    }
    public void cycleIntake(int height){

        robot.followTrajSeq(intake2[0]);
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.6);
        robot.resetAuto();
    }
    public void cycleIntake2(int height){
        robot.followTrajSeq(intake2[1]);
        lastCycle = robot.triggered();
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.6);
        robot.resetAuto();
    }
    public void cycleDrop(int i){
        if(i==1&&robot.triggered()){
            intakey=true;
        }
        robot.queuer.waitForFinish();
        robot.followTrajSeq(drop[i]);
        robot.queuer.addDelay(0.7);
        robot.grabAuto();
        robot.lowAuto(false);
        robot.drop(45.5);
    }
    public void pre(){
        robot.followTrajSeq(droppy[bark]);
        if (bark == 2) {
            robot.lowAuto(true);
            robot.yellowAuto(true);

        }else if(bark==1){
            robot.lowAuto(false);
            robot.yellowAuto(false);

        }
        else{
            robot.lowAuto(false);
            robot.yellowAuto(false);
        }
    }

    public void park(){
        robot.followTrajSeq(park[0]);
        robot.queuer.addDelay(.7);
        robot.resetAuto();
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
    }

    public void update(){
        robot.update();
        robot.queuer.setFirstLoop(false);
    }

    public boolean isAutDone(){
        return !robot.queuer.isFullfilled()&&time<29.8;
    }

    public void loop(){
        if ((time<20||lastCycle)&&!joever) {
            purp();
            pre();
            intake(5);
            cycleDrop(0);
            cycleIntake2(3);
            cycleDrop(1);
        }
        else if(!joever){
            joever = true;
        }
        if(joever && !lastCycle ){
            purp();
            pre();
            intake(5);
            cycleDrop(0);
            cycleIntake2(3);
            cycleDrop(1);
        }
        else if(joever && lastCycle && !parky){
            robot.queuer.reset();
            parky=true;
        }

        park();
        update();
    }


}
