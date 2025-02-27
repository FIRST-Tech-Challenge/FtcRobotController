package org.firstinspires.ftc.masters.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.ITDCons;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outtake;
import org.firstinspires.ftc.masters.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.masters.pedroPathing.constants.LConstants;

import java.util.List;

@Autonomous(name="Sample")
public class Sample extends LinearOpMode {


    Pose startPose = new Pose(10,110,0);
    Pose bucketPose = new Pose (19,124, Math.toRadians(-45));
    Pose sample1 = new Pose(20,128,Math.toRadians(-11));
    Pose sample2 = new Pose(20,128,Math.toRadians(-5));
    Pose sample3 = new Pose(20, 128, Math.toRadians(23));

    PathChain scorePreload, pickupSample1, pickupSample2, pickupSample3, scoreSample1, scoreSample2, scoreSample3;

    Follower follower;
    GoBildaPinpointDriver pinpoint;
    Servo led;



    enum PathState {ToBucket, Sample1, Sample2, Sample3, Score1, Score2, Score3 }

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        ElapsedTime elapsedTime = null;

        Init init = new Init(hardwareMap);
        Outtake outtake = new Outtake(init, telemetry);
        Intake intake = new Intake(init, telemetry);
        DriveTrain driveTrain = new DriveTrain(init, telemetry);
        outtake.setDriveTrain(driveTrain);
        outtake.setIntake(intake);
        intake.setOuttake(outtake);

        pinpoint = init.getPinpoint();
        led = init.getLed();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        outtake.initAutoSample();

        pinpoint.update();
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        telemetry.update();

        while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY){
            pinpoint.update();
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
            telemetry.update();
            led.setPosition(ITDCons.red);
            sleep(500);
        } if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY){
            pinpoint.update();
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
            telemetry.update();
            led.setPosition(ITDCons.green);
        }

        waitForStart();

        elapsedTime = null;
        PathState pathState =PathState.ToBucket;
        follower.followPath(scorePreload);
        outtake.scoreSample();


        while (opModeIsActive() && !isStopRequested()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            switch (pathState){
                case ToBucket:
                    telemetry.addData("ready?", outtake.isLiftReady());
                    if (!follower.isBusy() && outtake.isLiftReady() && elapsedTime==null){
                        outtake.openClaw();
                        intake.extendSlideHalfAuto();
                        elapsedTime = new ElapsedTime();
                    } else if (!follower.isBusy()  && elapsedTime!=null && elapsedTime.milliseconds()> 400){
                        intake.pickupSampleYellow();
                        follower.followPath(pickupSample1);
                        pathState = PathState.Sample1;
                        elapsedTime =null;
                    }

                    break;
                case Sample1:
                    if (!follower.isBusy() && outtake.isTransferDone() && !outtake.isScoringDone()){
                        outtake.scoreSample();
                        follower.followPath(scoreSample1);
                        intake.extendSlideHalfAuto();
                        pathState= PathState.Score1;
                        elapsedTime= null;

                    } else if (!follower.isBusy() && !outtake.isTransferDone() && elapsedTime==null){
                        intake.extendSlideMAxAuto();
                        elapsedTime= new ElapsedTime();
                    }
                    break;
                case Score1:
                    if (!follower.isBusy() && outtake.isLiftReady() && elapsedTime==null){
                        outtake.openClaw();
                        intake.extendSlideHalfAuto();
                        elapsedTime = new ElapsedTime();
                    } else if (!follower.isBusy() && elapsedTime==null && elapsedTime.milliseconds()> 400){
                        intake.pickupSampleYellow();
                        follower.followPath(pickupSample2);
                        pathState = PathState.Sample2;
                        elapsedTime =null;
                    }
                    break;
                case Sample2:
                    if (!follower.isBusy() && outtake.isTransferDone() && !outtake.isScoringDone()){
                        outtake.scoreSample();
                        follower.followPath(scoreSample2);
                        intake.extendSlideHalfAuto();
                        pathState= PathState.Score2;

                    }
                    else if (!follower.isBusy() && !outtake.isTransferDone() && elapsedTime==null){
                        intake.extendSlideMAxAuto();
                        elapsedTime= new ElapsedTime();
                    }
                    break;
                case Score2:
                    if (!follower.isBusy() && outtake.isLiftReady() && !outtake.isScoringDone()) {
                        outtake.openClaw();
                        intake.extendSlideHalfAuto();
                        elapsedTime = new ElapsedTime();
                    } else if (!follower.isBusy() && outtake.isScoringDone() && elapsedTime.milliseconds()> 400){
                        intake.pickupSampleYellow();
                        follower.followPath(pickupSample3);
                        pathState = PathState.Sample3;
                        elapsedTime =null;
                    }
                    break;
                case Sample3:
                    if (!follower.isBusy() && outtake.isTransferDone() && !outtake.isScoringDone()){
                        outtake.scoreSample();
                        follower.followPath(scoreSample3);
                        intake.extendSlideHalfAuto();
                        pathState= PathState.Score3;

                    }
                    else if (!follower.isBusy() && !outtake.isTransferDone() && elapsedTime==null){
                        intake.extendSlideMAxAuto();
                        elapsedTime= new ElapsedTime();
                    }
                    break;
                case Score3:
                    if (!follower.isBusy() && outtake.isLiftReady() && !outtake.isScoringDone()) {
                        outtake.openClaw();
                        elapsedTime = new ElapsedTime();
                    } else if (!follower.isBusy() && outtake.isScoringDone() && elapsedTime.milliseconds()> 400){
//                        intake.pickupSampleYellow();
//                        follower.followPath(pickupSample2);
//                        pathState = PathState.Sample2;
//                        elapsedTime =null;
                    }
                    break;
            }


            outtake.update();
            intake.update();
            follower.update();
            telemetry.addData("State", pathState);
            telemetry.update();
        }

    }

    protected void buildPaths(){

        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), bucketPose.getHeading())
                .build();

        pickupSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(sample1)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), sample1.getHeading())
                .build();


        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(bucketPose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        pickupSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(sample2)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), sample2.getHeading())
                .build();


        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(bucketPose)))
                .setLinearHeadingInterpolation(sample2.getHeading(), bucketPose.getHeading())
                .build();

        pickupSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(sample3)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), sample3.getHeading())
                .build();


        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(bucketPose)))
                .setLinearHeadingInterpolation(sample3.getHeading(), bucketPose.getHeading())
                .build();






    }
}
