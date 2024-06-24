package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.voltage;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.imuMultiply;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RR20 {
    boolean logi=false, isRight;
    LinearOpMode op;
    BradBot robot;
    int bark = 0, delaySec =0, barg=0;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] intake = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence[] drop = new TrajectorySequence[3];
    TrajectorySequence[] park = new TrajectorySequence[3];
    TrajectorySequence[] opark = new TrajectorySequence[3];





    public RR20(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(17,-64.5,toRadians(-90));
        robot.roadrun.setPoseEstimate(startPose);
        imuMultiply = 1.039 + .002*(robot.getVoltage()-12.5);
        spikey[0] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(10.5,-36,toRadians(0)), toRadians(-200))
                .build();

        spikey[1] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(13,-35,toRadians(-90)))
                .lineToLinearHeading(new Pose2d(13,-45,toRadians(-90)))
                .build();

        spikey[2] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(21,-38,toRadians(-90)))
                .lineToLinearHeading(new Pose2d(21,-50,toRadians(-90)))
                .build();

        if (!isLogi) {
            droppy[0] = robot.roadrun.trajectorySequenceBuilder(spikey[0].end())
                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(50))
                    .lineToLinearHeading(new Pose2d(38,-36,toRadians(-180)))
                    .lineToLinearHeading(new Pose2d(45.5, -30.5, toRadians(-180))).build();

            droppy[1] = robot.roadrun.trajectorySequenceBuilder(spikey[1].end())
                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                    .lineToLinearHeading(new Pose2d(38,-36,toRadians(-180)))
                    .lineToLinearHeading(new Pose2d(45.5, -35, toRadians(-180))).build();

            droppy[2] = robot.roadrun.trajectorySequenceBuilder(spikey[2].end())
//                    .lineToLinearHeading(new Pose2d(40,-50,toRadians(-180)))
                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                    .lineToLinearHeading(new Pose2d(38,-36,toRadians(-180)))
                    .lineToLinearHeading(new Pose2d(45.5, -41.5, toRadians(-180))).build();

        } else{
        }
        park[0] = robot.roadrun.trajectorySequenceBuilder(droppy[0].end())
                .lineTo(new Vector2d(droppy[0].end().getX()-3, droppy[0].end().getY()))
                .lineToLinearHeading(new Pose2d(44.5,-60, toRadians(-180)))
                .build();
        park[1] = robot.roadrun.trajectorySequenceBuilder(droppy[1].end())
                .lineTo(new Vector2d(droppy[1].end().getX()-3, droppy[1].end().getY()))
                .lineToLinearHeading(new Pose2d(44.5,-60, toRadians(-180)))
                .build();
        park[2] = robot.roadrun.trajectorySequenceBuilder(droppy[2].end())
                .lineTo(new Vector2d(droppy[2].end().getX()-3, droppy[2].end().getY()))
                .lineToLinearHeading(new Pose2d(44.5,-60, toRadians(-180)))
                .build();
        opark[0] = robot.roadrun.trajectorySequenceBuilder(droppy[0].end())
                .lineTo(new Vector2d(droppy[0].end().getX()-3, droppy[0].end().getY()))
                .lineToLinearHeading(new Pose2d(44.5,-20, toRadians(-180)))
                .build();
        opark[1] = robot.roadrun.trajectorySequenceBuilder(droppy[1].end())
                .lineTo(new Vector2d(droppy[1].end().getX()-3, droppy[1].end().getY()))
                .lineToLinearHeading(new Pose2d(44.5,-20, toRadians(-180)))
                .build();
        opark[2] = robot.roadrun.trajectorySequenceBuilder(droppy[2].end())
                .lineTo(new Vector2d(droppy[2].end().getX()-3, droppy[2].end().getY()))
                .lineToLinearHeading(new Pose2d(44.5,-20, toRadians(-180)))
                .build();

    /*    robot.dropServo(1);
        robot.dropServo(0);*/
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
            op.telemetry.addData("delaySec", delaySec);
            op.telemetry.addData("park,0=R,1=L", barg);
            if (gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "addSecs")) {
                delaySec++;
            }
            if (gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "minusSecs")) {
                delaySec = min(0, delaySec - 1);
            }
            if (gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "parkRight")) {
                barg = 0;
            }
            if (gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "parkLeft")) {
                barg = 1;
            }
            robot.update();
        }
        op.resetRuntime();
        time=0;
    }
    public void purp()
    {
//        bark=2;
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(delaySec);
        robot.queuer.waitForFinish();
        robot.followTrajSeq(spikey[bark]);
    }

    public void pre(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(droppy[bark]);
        if(bark==0) {
            robot.lowAuto(true);
            robot.yellowAuto(true);
            robot.drop(44);
        }
        else if(bark==1) {
            robot.lowAuto(true);
            robot.yellowAuto(true);
            robot.drop(44);
        }
        else {
            robot.lowAuto(true);
            robot.yellowAuto(true);
            robot.drop(47.75);
        }
    }

    public void park(){
        if(barg == 0){
            robot.followTrajSeq(park[bark]);
        }
        else{
            robot.followTrajSeq(opark[bark]);
        }
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
        return !robot.queuer.isFullfilled()&&time<29.8;
    }
}
