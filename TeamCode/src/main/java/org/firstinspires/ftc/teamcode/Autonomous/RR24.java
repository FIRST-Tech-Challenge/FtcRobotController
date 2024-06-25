package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFAprilCam.CONST;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFAprilCam.Y_OFFSET;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.Robots.BradBot.intakeFInishTIme;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.funnyIMUOffset;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.imuMultiply;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFAprilCam;
import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Config
public class RR24 {
    boolean logi=false, isRight,ultras = true, check = true, everChecked = false, intakey = false ;
    LinearOpMode op;
    BradBot robot;
    int bark = 0, delaySec =0;
    boolean lastCycle = false;
    double travelTime =3.5, lingerTime = 3;
    public static int overBark =0;
    TrajectorySequence altPark ;
    double[][] ranges = {{0,0},{0,0},{0,0},{0,0},{0,0}};
    int currentRange=0, currentSection=0;
    boolean joever = false;
    boolean parky = false;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] intake = new TrajectorySequence[3];
    TrajectorySequence[] intake2 = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence[] drop = new TrajectorySequence[3];
    TrajectorySequence[] park= new TrajectorySequence[3], parkLeft= new TrajectorySequence[3];





    public RR24(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(15,-61.5,toRadians(-90));
        robot.roadrun.setPoseEstimate(startPose);
        LATERAL_MULTIPLIER = 1.1;
        lastCycle = false;
        joever = false;
        parky = false;
        robot.queuer.setFirstLoop(true);

        spikey[0] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(9.5, -37, toRadians(0)), toRadians(180))
                .build();

        spikey[1] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12.5, -32.5, toRadians(-91)))
                .lineToLinearHeading(new Pose2d(12.5, -40.5, toRadians(-91)))
                .build();

        spikey[2] = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(19.5,-42, toRadians(-90)))
                .lineToLinearHeading(new Pose2d(19.5,-54, toRadians(-90)))
                .build();
        if (!isLogi) {
            droppy[0] = robot.roadrun.trajectorySequenceBuilder(spikey[0].end())
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(40, -31, toRadians(-179.9)))
                    .addTemporalMarker(robot::stopAllMotors)
                    .addTemporalMarker(robot::startIMU)
                    .addTemporalMarker(()->robot.roadrun.changeIMUInterval())
                    .lineToLinearHeading(new Pose2d(47, -29, toRadians(-179.9)))
                    .addTemporalMarker(()->robot.dropServo(0))
                    .addTemporalMarker(()->robot.dropServo(1))

                    .build();

            droppy[1] = robot.roadrun.trajectorySequenceBuilder(spikey[1].end())
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(38, -35, toRadians(-179.9)))
                    .addTemporalMarker(robot::stopAllMotors)
                    .addTemporalMarker(robot::startIMU)
                    .addTemporalMarker(()->robot.roadrun.changeIMUInterval())
                    .lineToLinearHeading(new Pose2d(46.5, -37, toRadians(-179.9)))
                    .addTemporalMarker(()->robot.dropServo(0))
                    .addTemporalMarker(()->robot.dropServo(1))
                    .build();

            droppy[2] = robot.roadrun.trajectorySequenceBuilder(spikey[2].end())
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(38, -36, toRadians(-179.9)))
                    .addTemporalMarker(robot::stopAllMotors)
                    .addTemporalMarker(robot::startIMU)
                    .addTemporalMarker(()->robot.roadrun.changeIMUInterval())
                    .lineToLinearHeading(new Pose2d(46.5, -40, toRadians(-179.9)))
                    .addTemporalMarker(()->robot.dropServo(0))
                    .addTemporalMarker(()->robot.dropServo(1))
                    .build();
        } else{


        }
        intake[0] = robot.roadrun.trajectorySequenceBuilder(droppy[0].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, -11.75), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(28,5,14))
                .splineToConstantHeading(new Vector2d(7, -11.75), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, -12.75), toRadians(180))
                .splineToConstantHeading(new Vector2d(-56,-13.75), toRadians(180))
                .build();
        intake[1] = robot.roadrun.trajectorySequenceBuilder(droppy[1].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, -11.75), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(28,5,14))
                .splineToConstantHeading(new Vector2d(7, -11.75), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, -12.75), toRadians(180))
                .splineToConstantHeading(new Vector2d(-56, -13.75), toRadians(180))
                .build();
        intake[2] = robot.roadrun.trajectorySequenceBuilder(droppy[2].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, -11.75), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(28,5,14))
                .splineToConstantHeading(new Vector2d(7, -11.75), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, -12.75), toRadians(180))
                .splineToConstantHeading(new Vector2d(-56, -13.75), toRadians(180))
                .build();
        drop[0] = robot.roadrun.trajectorySequenceBuilder(intake[0].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, -11.75), toRadians(0))
                .splineToConstantHeading(new Vector2d(18, -12.25), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(31))
//                .splineToConstantHeading(new Vector2d(40, -34), toRadians(0))
                .splineToConstantHeading(new Vector2d(46, -34), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(42))
                .build();

        intake2[0] = robot.roadrun.trajectorySequenceBuilder(drop[0].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, -10.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30,5,14))
                .splineToConstantHeading(new Vector2d(7, -10.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, -11.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-56.4, -11.85), toRadians(180))
                .build();
        drop[1] = robot.roadrun.trajectorySequenceBuilder(intake2[0].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, -11.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(10, -11.25), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(33))
//                .splineToConstantHeading(new Vector2d(40, -33.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(45.5, -33.5), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(42))
                .build();
        intake2[1] = robot.roadrun.trajectorySequenceBuilder(drop[1].end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25, -12.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25,5,14))
                .splineToConstantHeading(new Vector2d(7, -12.25), toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85,5,14))
                .splineToConstantHeading(new Vector2d(-30, -12.25), toRadians(180))
                .splineToConstantHeading(new Vector2d(-57.2, -12.25), toRadians(180))
                .build();
        altPark = robot.roadrun.trajectorySequenceBuilder(intake2[0].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30, -11.25), toRadians(0))
                .splineToConstantHeading(new Vector2d(20, -11.25), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                .splineToConstantHeading(new Vector2d(45, -13), toRadians(0))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(42))
                .build();

        park[0] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(42,-35, toRadians(180)))
                .lineToLinearHeading(new Pose2d(45,-13 , toRadians(-180)))
                .build();


        robot.setRight(true);
        robot.setBlue(false);
        robot.observeSpike();
        robot.hoverArm();
    }
    public void waitForStart(){
        CONST = -.015;
        funnyIMUOffset=0;
//        Y_OFFSET = 1.6;
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
            robot.update();
        }
        op.resetRuntime();

//        bark=2;
        time=0;
    }
    public void purp()
    {
//        bark=overBark;
//        if(bark==0){
//            funnyIMUOffset = 2.3; 0 1.2
//        }
//        if(bark==1){
//            funnyIMUOffset = 2.0; 1
//        }
//        if(bark ==2){
//            funnyIMUOffset = 2.3; 0.5
//        }
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
        lastCycle = robot.triggered();
        robot.followTrajSeq(intake2[0]);
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
        LOGGER.log("intakeFInTIme" + intakeFInishTIme);
        LOGGER.log("delTIme" + delTime);
        if(arriveTime>=29.75&& !robot.queuer.isNextExecuted()){
            drop[i] = altPark;
            delTime=0;
        }
        robot.queuer.queue(false,true);
        robot.queuer.addDelay(delTime);
        robot.followTrajSeq(drop[i]);
        robot.queuer.addDelay(0.7);
        robot.grabAuto();
        if(i==0)
            robot.louAuto(false);
        else
            robot.lowAuto(false);
        robot.drop(46);
    }
    public void pre(){

        robot.followTrajSeq(droppy[bark]);
        if (bark == 2) {
            robot.lowAuto(false);
            robot.yellerAuto(false);

        }else if(bark==1){
            robot.lowAuto(true);
            robot.yellerAuto(true);

        }
        else{
            robot.lowAuto(true);
            robot.yellerAuto(true);
        }
    }

    public void park(){
        if(currentPose.vec().distTo(park[0].end().vec())>1 || robot.roadrun.isBusy())
            robot.followTrajSeq(park[0]);
        else
            robot.queuer.queue(false,true);
        robot.queuer.addDelay(.3);
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
        if ((time<21||lastCycle)&&!joever) {
            purp();
            pre();
            intake(5);
            cycleDrop(0);
            if(!drop[0].equals(altPark)) {
                cycleIntake2(3);
                cycleDrop(1);
            }
            else{
                robot.queuer.reset();
                joever = true;
            }
        }
        else if(!joever){
            joever = true;
        }
        if(joever && !lastCycle ){
            purp();
            pre();
            intake(5);
            cycleDrop(0);
            if(!drop[0].equals(altPark)) {
                cycleIntake2(3);
                cycleDrop(1);
            }
            else{
                robot.queuer.reset();
                joever = true;
                lastCycle
                        = true;
            }

        }
        else if(joever && lastCycle && !parky){
            robot.queuer.reset();
            parky=true;
        }

        park();
        update();
    }


}