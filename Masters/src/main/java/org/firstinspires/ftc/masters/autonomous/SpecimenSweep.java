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

import org.firstinspires.ftc.masters.components.ITDCons;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outtake;
import org.firstinspires.ftc.masters.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.masters.pedroPathing.constants.LConstants;

import java.util.List;

//position are setup with pedro coordinate from blue side
//auto can be used for blue and red

@Autonomous(name="SpecimenSweep")
public class SpecimenSweep extends LinearOpMode {

    Pose startPose = new Pose(10,45,0);
    Pose scoringPose = new Pose(38,70.5, 0);

    Pose startSweepPose1 = new Pose (35,51, Math.toRadians(-45));
    Pose endSweepPose1 = new Pose (36, 50,  Math.toRadians(-45-90));

    Pose startSweep2 = new Pose (28, 45,Math.toRadians(-45));
    Pose endSweep2 = new Pose(27, 46, Math.toRadians(-45-90));

    Pose startSweepPose3 = new Pose (36,30, Math.toRadians(314-360));
    Pose endSweepPose3 = new Pose (28+10, -24+45,  Math.toRadians(243-360));

    Pose midPoint0 = new Pose(30, 60, 0);
    Pose midPoint1 = new Pose(20,25,0);
    Pose midPoint2 = new Pose(60,36,0);

    Pose pickupPose = new Pose (0+10,-13+45, 0);
    Pose pickupMid = new Pose(23+10,26+45,0);
    Pose pushPose1 = new Pose(65,28,0);
    Pose endPushPose1 = new Pose (22,28,0);
    Pose pushPose2 = new Pose(65,17,0);
    Pose endPushPose2 = new Pose(22,17,0);
    Pose pushPose3 = new Pose(65,11,0);
    Pose endPushPose3 = new Pose(22,11,0);

    Path scorePreload, pickup1, score, towall, tosub, toSweep1, sweep1, toSweep2, sweep2,
    toSweep3,sweep3
    ;
    PathChain pushSample1, pushSample2, pushSample3, pickUp;

    int cycleCount = 1;

    enum PathState {Lift,Start,ToSub, ScorePreload,Sample1,PushSample1, Sample2, PushSample2, Sample3, PushSample3, PickUpSpec, GrabSpec3, ScoreSpec3, Score, End}

    Follower follower;

    GoBildaPinpointDriver pinpoint;
    Servo led;

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
//        DriveTrain driveTrain = new DriveTrain(init, telemetry);

        pinpoint = init.getPinpoint();
        led = init.getLed();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        PathState state = PathState.Start;
        outtake.initAutoSpecimen();
        intake.retractSlide();

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


        intake.extendSlideMax();

        follower.followPath(toSweep1);

        elapsedTime = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            switch (state){
                case Start:
                    if (!follower.isBusy()){
                        follower.followPath(sweep1);
                        state= PathState.Sample1;
                    }
                    intake.servoToDrop();
                    break;

                case Sample1:
                    if (!follower.isBusy()){
                        follower.followPath(toSweep2);
                        intake.servoToNeutral();
                        state = PathState.Sample2;
                    }
                    break;
                case Sample2:
                    if (!follower.isBusy()){
                        follower.followPath(sweep2);
                        intake.servoToDrop();
                        state = PathState.Sample3;
                    }
                    break;
                case Sample3:
                    if (!follower.isBusy()){
                        follower.followPath(toSweep3);
                        intake.servoToDrop();
                        state = PathState.PushSample3;
                    }
                    break;
                case PushSample3:
                    if(!follower.isBusy()){
                        follower.followPath(sweep3);

                    }
                    break;

//                case Sample2:
//                    if (elapsedTime.milliseconds()>1500){
//                        state= PathState.Sample3;
//                        follower.followPath(sweep1);
//                    }
//                    if (elapsedTime.milliseconds()>1500){
//                        if (pinpoint.getHeading()>-Math.toRadians(115)) {
//                            driveTrain.turn(0.7);
//                        } else {
//                            driveTrain.drive(0);
//                        }
//                    }
               //     break;
//                case Sample3:
//                    if (!follower.isBusy()){
//                        follower.followPath(toSweep2);
//                        intake.servoToNeutral();
//                    }
//                    break;
//                case PickUpSpec:
//                    if (!follower.isBusy()){
//                        if (elapsedTime==null) {
//                            outtake.closeClaw();
//                            elapsedTime= new ElapsedTime();
//                        } else if (elapsedTime.milliseconds()>500){
//                            follower.followPath(score);
//                            outtake.scoreSpecimen();
//                            elapsedTime=null;
//                            if(cycleCount <= 4) {
//                                state = PathState.ScoreSpec3;
//                            }
//                            if(cycleCount > 4) {
//                                state = PathState.Score;
//                            }
//                        }
//                    }
//                    break;
//                case ScoreSpec3:
//                    if (!follower.isBusy()){
//                        if (elapsedTime==null) {
//                            outtake.openClawAuto();
//                            cycleCount++;
//                            elapsedTime= new ElapsedTime();
//                        } else if (elapsedTime.milliseconds()>350){
//                            follower.followPath(towall);
//                            outtake.closeClaw();
//                            outtake.moveToPickUpFromWall();
//                            elapsedTime = null;
//                            state = PathState.PickUpSpec;
//                        }
//                    }
//                    break;
//
////                case Score:
////                    if (!follower.isBusy()){
////                        if (elapsedTime==null) {
////                            //outtake.openClaw();
////                            elapsedTime= new ElapsedTime();
////
////                        } else if (elapsedTime.milliseconds()>150){
////                            follower.followPath(pickUp);
////                            //outtake.moveToPickUpFromWall();
////                            elapsedTime=null;
////                            state= PathState.End;
////                        }
////                    }
////                    break;
////                case End:
////                    if (!follower.isBusy()){
////                        //outtake.setTarget(0);
////                    }
////                    break;
            }

            outtake.update();
            intake.update();
            follower.update();
        }
    }

    protected void buildPaths(){

        toSweep1 = new Path(new BezierLine(new Point(startPose),new Point(startSweepPose1) ));
        toSweep1.setLinearHeadingInterpolation(0, startSweepPose1.getHeading());

        sweep1 = new Path(new BezierCurve(new Point(startSweepPose1), new Point(endSweepPose1)));
        sweep1.setLinearHeadingInterpolation(startSweepPose1.getHeading(), endSweepPose1.getHeading());

        toSweep2 = new Path(new BezierCurve(new Point(endSweepPose1), new Point(startSweep2)));
        toSweep2.setLinearHeadingInterpolation(endSweepPose1.getHeading(), startSweep2.getHeading());

        sweep2 = new Path(new BezierCurve(new Point(startSweep2), new Point(endSweep2)));
        sweep2.setLinearHeadingInterpolation(startSweep2.getHeading(), endSweep2.getHeading());

        toSweep3 = new Path(new BezierCurve(new Point(endSweep2), new Point(startSweepPose3)));
        toSweep3.setLinearHeadingInterpolation(endSweep2.getHeading(), startSweepPose3.getHeading());

        sweep3 = new Path(new BezierCurve(new Point(startSweepPose3), new Point(endSweepPose3)));
        sweep3.setLinearHeadingInterpolation(startSweepPose3.getHeading(), endSweepPose3.getHeading());

        pickup1 = new Path(new BezierCurve(new Point(endPushPose2), new Point(pickupMid), new Point(pickupPose)));
        pickup1.setLinearHeadingInterpolation(endPushPose2.getHeading(), pickupPose.getHeading());

        score = new Path(new BezierLine(new Point(pickupPose), new Point(scoringPose.getX(), scoringPose.getY()+2)));
        score.setLinearHeadingInterpolation(pickupPose.getHeading(), scoringPose.getHeading());

        towall = new Path(new BezierCurve(new Point(scoringPose.getX(), scoringPose.getY()+2), new Point(pickupPose)));
        towall.setLinearHeadingInterpolation(scoringPose.getHeading(), pickupPose.getHeading());

        pickUp = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoringPose), new Point(pickupPose), new Point(pickupPose)))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), pickupPose.getHeading())
                .build();

    }
}
