package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.voltage;
import static org.firstinspires.ftc.teamcode.Robots.BradBot.intakeFInishTIme;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.imuMultiply;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BL20 {
    boolean logi=false, isRight;
    LinearOpMode op;
    BradBot robot;
    int bark = 0, delaySec =0, barg=0;
    int lingerTime = 4;
    double travelTime = 2;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] intake = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence[] drop = new TrajectorySequence[3];
    TrajectorySequence[] park = new TrajectorySequence[3];
    TrajectorySequence[] opark = new TrajectorySequence[3];
    double[][] ranges = {{0,0},{0,0},{0,0},{0,0},{0,0}};
    int currentRange =0;
    int currentSection=0;
    TrajectorySequence[] droppa = new TrajectorySequence[3];




    public BL20(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(-37.5,61.5,toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);
        imuMultiply = 1.039 + .002*(robot.getVoltage()-12.5);
        spikey[0] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-31,32,toRadians(180)),toRadians(-30))
                .build();

        spikey[1] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-36,32,toRadians(90)))
                .build();

        spikey[2] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-42,36,toRadians(90)))
                .build();
        droppa[0] = robot.roadrun
                .trajectorySequenceBuilder(spikey[0].end())
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(-40,38,toRadians(180)))
                .lineToLinearHeading(new Pose2d(-40,58,toRadians(180)))
                .lineToLinearHeading(new Pose2d(27,58,toRadians(180)))
                .addTemporalMarker(()->{intakeFInishTIme=time;}).build();
        droppa[1] = robot.roadrun
                .trajectorySequenceBuilder(spikey[1].end())
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(-40,38,toRadians(180)))
                .lineToLinearHeading(new Pose2d(-40,58,toRadians(180)))
                .lineToLinearHeading(new Pose2d(27,58,toRadians(180)))
                .addTemporalMarker(()->{intakeFInishTIme=time;}).build();
        droppa[2] = robot.roadrun
                .trajectorySequenceBuilder(spikey[2].end())
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(-40,38,toRadians(180)))
                .lineToLinearHeading(new Pose2d(-40,58,toRadians(180)))
                .lineToLinearHeading(new Pose2d(27,58,toRadians(180)))
                .addTemporalMarker(()->{intakeFInishTIme=time;}).build();

        if (!isLogi) {
            droppy[0] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(droppa[0].end())
                            .lineToLinearHeading(new Pose2d(38, 36, toRadians(-180)))
                            .lineToLinearHeading(new Pose2d(46.5,42,toRadians(180)))
                            .build();

            droppy[1] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(droppa[0].end())
                            .lineToLinearHeading(new Pose2d(38, 36, toRadians(-180)))
                            .lineToLinearHeading(new Pose2d(46.5,34.5,toRadians(180)))
                            .build();

            droppy[2] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(droppa[0].end())
                            .lineToLinearHeading(new Pose2d(38, 36, toRadians(-180)))
                            .lineToLinearHeading(new Pose2d(46.5,31.5,toRadians(180)))
                            .build();

        } else{
        }

        park[0] = robot.roadrun.trajectorySequenceBuilder(droppy[0].end())
                .lineTo(new Vector2d(droppy[0].end().getX()-3, droppy[0].end().getY()))
                .lineToLinearHeading(new Pose2d(43,57, toRadians(180)))
                .build();
        park[1] = robot.roadrun.trajectorySequenceBuilder(droppy[1].end())
                .lineTo(new Vector2d(droppy[1].end().getX()-3, droppy[1].end().getY()))
                .lineToLinearHeading(new Pose2d(43,57, toRadians(180)))
                .build();
        park[2] = robot.roadrun.trajectorySequenceBuilder(droppy[2].end())
                .lineTo(new Vector2d(droppy[2].end().getX()-3, droppy[2].end().getY()))
                .lineToLinearHeading(new Pose2d(43,57, toRadians(180)))
                .build();
        opark[0] = robot.roadrun.trajectorySequenceBuilder(droppy[0].end())
                .lineTo(new Vector2d(droppy[0].end().getX()-3, droppy[0].end().getY()))
                .lineToLinearHeading(new Pose2d(43,20, toRadians(180)))
                .build();
        opark[1] = robot.roadrun.trajectorySequenceBuilder(droppy[1].end())
                .lineTo(new Vector2d(droppy[1].end().getX()-3, droppy[1].end().getY()))
                .lineToLinearHeading(new Pose2d(43,20, toRadians(180)))
                .build();
        opark[2] = robot.roadrun.trajectorySequenceBuilder(droppy[2].end())
                .lineTo(new Vector2d(droppy[2].end().getX()-3, droppy[2].end().getY()))
                .lineToLinearHeading(new Pose2d(43,20, toRadians(180)))
                .build();

//    robot.dropServo(1);
//    robot.dropServo(0);
        robot.setRight(false);
        robot.setBlue(true);
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
            boolean up = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "addSecs")
                    ,down = gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "minusSecs")
                    , right = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "parkRight"),
                    left = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "parkLeft"),
                    a = op.gamepad1.a,
                    b = op.gamepad1.b;
            if(a&&up){
                currentRange ++;
                if(currentRange>ranges.length){
                    currentRange =0;
                }
            }
            else if(a&&down){
                currentRange--;
                if(currentRange<0){
                    currentRange = ranges.length-1;
                }
            }
            else if(a && left){
                currentSection--;
                if(currentSection<0){
                    currentSection = 1;
                }
            }
            else if(a&&right){
                currentSection ++;
                if(currentSection>1){
                    currentSection=0;
                }
            }
            else if(b&&up){
                ranges[currentRange][currentSection]++;
            }
            else if(b&&down){
                if(ranges[currentRange][currentSection]>0){
                    ranges[currentRange][currentSection]--;
                }
            }
            else if(b&&right){
                ranges[currentRange][currentSection]+=5;
            }
            else if(b&&left){
                if(ranges[currentRange][currentSection]>4){
                    ranges[currentRange][currentSection]-=5;
                }
            }
            else if (right) {
                isRight = true;
            }
            else if (left) {
                isRight = false;
            }
            String stringify = "";
            for(double[] i : ranges){
                stringify += "["+i[0]+","+i[1]+"]";
            }
            op.telemetry.addData("ranges", stringify);
            packet.put("ranges", stringify);
            if (gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "parkRight")) {
                barg=1;
            }
            if (gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "parkLeft")) {
                barg=0;
            }
            robot.update();
        }
        op.resetRuntime();
        time=0;
    }
    public void purp()
    {
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(delaySec);
        robot.queuer.waitForFinish();
        robot.followTrajSeq(spikey[bark]);
    }

    public void pre(){
        robot.followTrajSeq(droppa[bark]);
        robot.queuer.waitForFinish();
        double delTime = 0;
        double arriveTime = intakeFInishTIme+travelTime;
        double leaveTime = arriveTime+lingerTime;
        for(var j : ranges){
            if(arriveTime>j[0]&&arriveTime<j[1])
                delTime = j[1]-arriveTime;
            if(leaveTime>j[0]&&leaveTime<j[1]){
                delTime = max(j[1]-arriveTime,delTime);
            }
        }
        arriveTime = arriveTime+delTime;
        LOGGER.log("arriveTIme" + arriveTime);
        robot.queuer.addDelay(delTime);
        robot.followTrajSeq(droppy[bark]);
        if(bark==0) {
            robot.lowAuto(true);
            robot.yellowAuto(true);
            robot.drop(45.5);
        }
        else if(bark==1){
            robot.lowAuto(true);
            robot.yellowAuto(true);
            robot.drop(45);
        }
        else {
            robot.lowAuto(false);
            robot.yellowAuto(false);
            robot.drop(45.5);
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
