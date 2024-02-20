package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BL23 {
    boolean logi=false, isRight;
    LinearOpMode op;
    BradBot robot;
    int bark = 0, delaySec =0;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] intake = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence[] drop = new TrajectorySequence[3];
    TrajectorySequence park, parkRight;





    public BL23(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(-36,60.5,toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);

        spikey[0] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-48, 27.5,toRadians(120)), toRadians(-90))
                .lineToLinearHeading(new Pose2d(-49, 34.5,toRadians(120)))

                .build();

        spikey[1] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-39,36,toRadians(90)))
                .addTemporalMarker(robot::done)
                .build();

        spikey[2] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-33,38, toRadians(180)), toRadians(0))
                .build();
        intake[0] = robot.roadrun
                .trajectorySequenceBuilder(spikey[0].end())
                .lineToLinearHeading(new Pose2d(-51, 37.5,toRadians(120)))
                .lineToLinearHeading(new Pose2d(-50.5,34.25, toRadians(190)))
                .build();
        intake[1] = robot.roadrun
                .trajectorySequenceBuilder(spikey[1].end())
                .lineToLinearHeading(new Pose2d(-51.5,32.25, toRadians(185)))
                .addTemporalMarker(robot::done)
                .build();
        intake[2] = robot.roadrun
                .trajectorySequenceBuilder(spikey[2].end())
                .lineToLinearHeading(new Pose2d(-50.5,32.25, toRadians(180)))
                .addTemporalMarker(robot::done)
                .build();


        if (!isLogi) {
            droppy[0] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(intake[1].end())
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(-40, 56.5), toRadians(15))
                            .splineToConstantHeading(new Vector2d(-20, 57.5), toRadians(0))
                            .splineToConstantHeading(new Vector2d(25, 57.5), toRadians(-2))
                            .splineToConstantHeading(new Vector2d(47, 29), toRadians(0))
//                            .addTemporalMarker(robot::done)
                            .build();


            droppy[1] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(intake[1].end())
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(-40, 56.5), toRadians(15))
                            .splineToConstantHeading(new Vector2d(-20, 57.5), toRadians(0))
                            .splineToConstantHeading(new Vector2d(20, 57.5), toRadians(-2))
                            .splineToConstantHeading(new Vector2d(47, 35), toRadians(0))
//                            .addTemporalMarker(robot::done)
                            .build();

            droppy[2] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(intake[1].end())
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(-40, 56.5), toRadians(15))
                            .splineToConstantHeading(new Vector2d(-20, 57.5), toRadians(0))
                            .splineToConstantHeading(new Vector2d(20, 57.5), toRadians(-2))
                            .splineToConstantHeading(new Vector2d(46, 38), toRadians(0))
//                            .addTemporalMarker(robot::done)
                            .build();

        } else{


        }

        backToStack[0] = robot.roadrun
                .trajectorySequenceBuilder(droppy[0].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(20, 57.5), toRadians(180))
//                .splineTo(new Vector2d(10, -57.5), toRadians(186))
                .splineToConstantHeading(new Vector2d(-25, 58), toRadians(180))
                .splineToConstantHeading(new Vector2d(-51, 27.25), toRadians(180))
                .addTemporalMarker(robot::done)
                .build();
        backToStack[1] = robot.roadrun
                .trajectorySequenceBuilder(droppy[1].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(20, 57.5), toRadians(180))
//                .splineTo(new Vector2d(10, -57.5), toRadians(186))
                .splineToConstantHeading(new Vector2d(-25, 58), toRadians(180))
                .splineToConstantHeading(new Vector2d(-51, 30.25), toRadians(180))
                .addTemporalMarker(robot::done)
                .build();
        backToStack[2] = robot.roadrun
                .trajectorySequenceBuilder(droppy[2].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(20, 57.5), toRadians(180))
//                .splineTo(new Vector2d(10, -57.5), toRadians(186))
                .splineToConstantHeading(new Vector2d(0, 58), toRadians(180))

                .splineToConstantHeading(new Vector2d(-26, 57), toRadians(180))
                .splineToConstantHeading(new Vector2d(-51, 30.25), toRadians(180))
                .addTemporalMarker(robot::done)
                .build();
        drop[0] = robot.roadrun.trajectorySequenceBuilder(backToStack[0].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, 53.5), toRadians(15))
                .splineToConstantHeading(new Vector2d(-20, 53.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(6, 56.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(47, 38), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        drop[1] = robot.roadrun.trajectorySequenceBuilder(backToStack[1].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, 53.5), toRadians(15))
                .splineToConstantHeading(new Vector2d(-20, 55.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(6, 56.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(47.5, 38), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        drop[2] = robot.roadrun.trajectorySequenceBuilder(backToStack[2].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, 53.5), toRadians(15))
                .splineToConstantHeading(new Vector2d(-20, 55.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(6, 56.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(47, 37), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
        park = robot.roadrun.trajectorySequenceBuilder(drop[1].end())
                .lineToLinearHeading(new Pose2d(43.8,40, toRadians(-180)))
                .lineToLinearHeading(new Pose2d(45, 60,toRadians(-180)))
                .build();
        parkRight = robot.roadrun.trajectorySequenceBuilder(drop[1].end())
                .lineToLinearHeading(new Pose2d(43.8,20, toRadians(-180)))
                .lineToLinearHeading(new Pose2d(45, 6,toRadians(-180)))
                .build();
//
//
//
//        parky[0] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[0].end())
//                .lineToLinearHeading(new Pose2d(43.3,-29,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(55,-60,toRadians(-180)))
//                .build();
//
//        parky[1] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[1].end())
//                .lineToLinearHeading(new Pose2d(43.3,-35.5,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(55,-60,toRadians(-180)))
//                .build();
//
//        parky[2] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[2].end())
//                .lineToLinearHeading(new Pose2d(43.3,-41.5,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(55,-60,toRadians(-180)))
//                .build();/

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
        time=0;
    }
    public void purp()
    {
        robot.queuer.queue(false, true);
        robot.upAuto();
        if (bark != 0) {
            robot.purpurAuto();
        }
        else{
            robot.purpurAuto2();
        }
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
    public void cycleIntake(int height){

        robot.followTrajSeq(backToStack[bark]);
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.3);
        robot.resetAuto();
    }
    public void cycleDrop(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(drop[bark]);
        robot.grabAuto();
        robot.lowAuto();
        robot.drop();
    }
    public void pre(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(droppy[bark]);
        robot.grabAuto();
        robot.lowAuto();
        if (bark == 2) {
            robot.yellowAuto(true);
        }else{
            robot.yellowAuto(false);

        }
        robot.drop();

    }

    public void park(){
        robot.queuer.addDelay(.7);
        robot.resetAuto();
        if(isRight){
            robot.followTrajSeq(parkRight);
    } else {
      robot.followTrajSeq(park);
        }
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


}
