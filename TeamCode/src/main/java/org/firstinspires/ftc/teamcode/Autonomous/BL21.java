package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BL21 {
    boolean logi=false, isRight = false, ultras = true, check = false, everChecked = false;
    int delaySec = 0;
    LinearOpMode op;
    BradBot robot;
    int bark = 0;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] intake = new TrajectorySequence[3];

    TrajectorySequence[] pathy = new TrajectorySequence[3], pathyUltra = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];

    TrajectorySequence[] parky = new TrajectorySequence[3], parkLeft = new TrajectorySequence[3];



    public BL21(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false, isLogi);
        Pose2d startPose = new Pose2d(-36,60.5,toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);

        spikey[2] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-40, 18.5,toRadians(90)), toRadians(-90))
                .lineToLinearHeading(new Pose2d(-44, 36.5,toRadians(90)))
                .build();

        spikey[1] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-37,36,toRadians(90)))
                .addTemporalMarker(robot::done)
                .build();

        spikey[0] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-31,36, toRadians(180)), toRadians(0))
                .build();
        intake[2] = robot.roadrun
                .trajectorySequenceBuilder(spikey[2].end())
                .lineToLinearHeading(new Pose2d(-51, 37.5,toRadians(120)))
                .lineToLinearHeading(new Pose2d(-50,34.25, toRadians(200)))
                .build();
        intake[1] = robot.roadrun
                .trajectorySequenceBuilder(spikey[1].end())
                .lineToLinearHeading(new Pose2d(-50,32.25, toRadians(185)))
                .addTemporalMarker(robot::done)
                .build();
        intake[0] = robot.roadrun
                .trajectorySequenceBuilder(spikey[0].end())
                .lineToLinearHeading(new Pose2d(-50,32.25, toRadians(180)))
                .addTemporalMarker(robot::done)
                .build();


        pathy[0] = robot.roadrun
                .trajectorySequenceBuilder(intake[0].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-43,31, toRadians(180)))
                .lineToLinearHeading(new Pose2d(-38,58.5, toRadians(180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(10,58.5, toRadians(180)))
                .build();

        pathy[1] = robot.roadrun
                .trajectorySequenceBuilder(intake[1].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-38,58.5, toRadians(180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(15,58.5, toRadians(180)))
                .build();

        pathy[2] = robot.roadrun
                .trajectorySequenceBuilder(intake[2].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-38,59.5, toRadians(180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(15,59.5, toRadians(180)))
                .build();
        pathyUltra[0] = robot.roadrun
                .trajectorySequenceBuilder(intake[0].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-40,58.5, toRadians(180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(0,58.7, toRadians(180)))
                .lineToLinearHeading(new Pose2d(15,58, toRadians(130)))
                .build();

        pathyUltra[1] = robot.roadrun
                .trajectorySequenceBuilder(intake[1].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-46,58.5, toRadians(180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(0,58, toRadians(180)))

                .lineToLinearHeading(new Pose2d(15,58, toRadians(130)))
                .build();

        pathyUltra[2] = robot.roadrun
                .trajectorySequenceBuilder(intake[2].end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-46,59.5, toRadians(180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(0,60, toRadians(180)))
                .lineToLinearHeading(new Pose2d(15,60, toRadians(130)))
                .build();
        if(!isLogi){


            droppy[0] = robot.roadrun
                    .trajectorySequenceBuilder(pathy[0].end())
                    .lineToLinearHeading(new Pose2d(42.5,35,toRadians(180)))
                    .lineToLinearHeading(new Pose2d(46.3,41.5,toRadians(180)))
                    .build();
            droppy[1] = robot.roadrun
                    .trajectorySequenceBuilder(pathy[1].end())
                    .lineToLinearHeading(new Pose2d(42.5,35,toRadians(180)))
                    .lineToLinearHeading(new Pose2d(46.3,35.25,toRadians(180)))
                    .build();
            droppy[2] = robot.roadrun
                    .trajectorySequenceBuilder(pathy[2].end())
                    .lineToLinearHeading(new Pose2d(42.5,32,toRadians(180)))
                    .lineToLinearHeading(new Pose2d(46.3,33,toRadians(180)))
                    .build();
        }
        else{
            droppy[0] = robot.roadrun
                    .trajectorySequenceBuilder(pathy[0].end())
                    .lineToLinearHeading(new Pose2d(42.5,35,toRadians(180)))
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(46.8,41.5,toRadians(180)))
                    .build();
            droppy[1] = robot.roadrun
                    .trajectorySequenceBuilder(pathy[1].end())
                    .lineToLinearHeading(new Pose2d(42.5,35,toRadians(180)))
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(46.8,35.25,toRadians(180)))
                    .build();
            droppy[2] = robot.roadrun
                    .trajectorySequenceBuilder(pathy[2].end())
                    .lineToLinearHeading(new Pose2d(42.5,32,toRadians(180)))
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(46.8,29,toRadians(180)))
                    .build();
        }



//        droppy[1] = robot.roadrun
//                .trajectorySequenceBuilder(pathy[1].end())
//                .lineToLinearHeading(new Pose2d(0,0,toRadians(0)))
//                .build();
//
//        droppy[2] = robot.roadrun
//                .trajectorySequenceBuilder(pathy[2].end())
//                .lineToLinearHeading(new Pose2d(0,0,toRadians(0)))
//                .build();


        parky[0] = robot.roadrun
                .trajectorySequenceBuilder(droppy[0].end())
                .lineToLinearHeading(new Pose2d(43.3,41.5,toRadians(180)))

                .lineToLinearHeading(new Pose2d(50,9,toRadians(180)))
                .build();

        parky[1] = robot.roadrun
                .trajectorySequenceBuilder(droppy[1].end())
                .lineToLinearHeading(new Pose2d(43.3,35.5,toRadians(180)))
                .lineToLinearHeading(new Pose2d(50,9,toRadians(180)))
                .build();

        parky[2] = robot.roadrun
                .trajectorySequenceBuilder(droppy[2].end())
                .lineToLinearHeading(new Pose2d(43.3,29.5,toRadians(180)))
                .lineToLinearHeading(new Pose2d(50,9,toRadians(180)))
                .build();

        parkLeft[0] = robot.roadrun
                .trajectorySequenceBuilder(droppy[0].end())
                .lineToLinearHeading(new Pose2d(43.3,41.5,toRadians(180)))

                .lineToLinearHeading(new Pose2d(50,9,toRadians(180)))
                .build();

        parkLeft[1] = robot.roadrun
                .trajectorySequenceBuilder(droppy[1].end())
                .lineToLinearHeading(new Pose2d(43.3,35.5,toRadians(180)))
                .lineToLinearHeading(new Pose2d(50,60,toRadians(180)))
                .build();

        parkLeft[2] = robot.roadrun
                .trajectorySequenceBuilder(droppy[2].end())
                .lineToLinearHeading(new Pose2d(43.3,29.5,toRadians(180)))
                .lineToLinearHeading(new Pose2d(50,60,toRadians(180)))
                .build();
        robot.dropServo(1);
        robot.setRight(true);
        robot.setBlue(false);
        robot.observeSpike();
        check = false;
        everChecked = false;
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
            if (gampad.readGamepad(op.gamepad1.x, "gamepad1_x", "ultras")) {
                ultras = !ultras;
            }
            robot.update();
        }
        op.resetRuntime();
        time=0;
    }
    public void purp()
    {
        robot.queuer.queue(false, true);
        robot.upAuto();
//        if (bark != 0) {
            robot.purpurAuto();
//        }
//        else{
//            robot.queuer.addDelay(3.0);
//            robot.purpurAuto2();
//        }
        if (bark == 0) {
            robot.queuer.addDelay(0.4+delaySec);
        } else {
            robot.queuer.addDelay(delaySec);
        }

        robot.followTrajSeq(spikey[bark]);
        robot.queuer.addDelay(0.0);
        robot.dropAuto(0);
    }

    public void intake(int height){
        robot.followTrajSeq(intake[bark]);
        robot.resetAuto();
        if (bark == 0) {
            robot.queuer.addDelay(2.0);
        }
        robot.intakeAuto(height);
    }
    public void pre(){
        robot.queuer.waitForFinish();
        robot.queuer.addDelay(0.3);
        if (ultras) {
            robot.followTrajSeq(pathyUltra[bark]);
        }
        else {
            robot.followTrajSeq(pathy[bark]);
        }
        robot.queuer.addDelay(.6);
        robot.resetAuto();
        robot.grabSupAuto();
        robot.followTrajSeq(droppy[bark]);
        if (bark == 2) {
            robot.lowAuto(true);
            robot.yellowAuto(true);
        }else{
            robot.lowAuto(false);
            robot.yellowAuto(false);

        }
        if (ultras) {
            robot.queuer.addDelay(0.2);
            robot.followTrajSeqUltra(check, droppy[bark].end());
            robot.queuer.waitForFinish();
            if(check){
                everChecked = true;
            }
            robot.followTrajSeq(droppy[bark], !everChecked);

        }

        robot.queuer.waitForFinish();
        robot.drop();
    }

    public void park(){

        if (ultras) {
            robot.queuer.waitForFinish();
        }
        if (isRight) {
            robot.followTrajSeq(parky[bark]);
        }
        else{
            robot.followTrajSeq(parkLeft[bark]);
        }
        robot.queuer.addDelay(.7);
        robot.resetAuto();
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
    }

    public void update(){
        if (ultras) {
            check = robot.checkAlliance();
        }
        robot.update();
        robot.queuer.setFirstLoop(false);
    }

    public boolean isAutDone(){
        return !robot.queuer.isFullfilled()&&time<29.8;
    }


}
