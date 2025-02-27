package org.firstinspires.ftc.masters.autonomous;

import com.pedropathing.follower.Follower;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.ITDCons;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outtake;
import org.firstinspires.ftc.masters.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.masters.pedroPathing.constants.LConstants;

import java.util.List;

//position are setup with pedro coordinate from blue side
//auto can be used for blue and red

@Autonomous(name="Specimen")
public class Specimen extends LinearOpMode {

    Pose startPose = new Pose(10,66,0);
    Pose scoringPose = new Pose(40,70.5, 0);

    Pose midPoint0 = new Pose(30, 60, 0);
    Pose midPoint1 = new Pose(20,25,0);
    Pose midPoint2 = new Pose(60,36,0);

    Pose pickupPose1 = new Pose(13, 40, 0);
    Pose pickupPose = new Pose (10.75,40, 0);
    Pose pushPose1 = new Pose(65,28,0);
    Pose endPushPose1 = new Pose (24,28,0);
    Pose pushPose2 = new Pose(65,17,0);
    Pose endPushPose2 = new Pose(24,17,0);
    Pose pushPose3 = new Pose(65,11,0);
    Pose endPushPose3 = new Pose(23,11,0);

    Path scorePreload, pickup1, score, towall, tosub;
    PathChain pushSample1, pushSample2, pushSample3, pickUp;

    int cycleCount = 1;

    enum PathState {Lift,Start,ToSub, ScorePreload,Sample1,PushSample1, Sample2, PushSample2, Sample3, PushSample3, PickUpSpec, GrabSpec3, ScoreSpec3, Score, End}

    Follower follower;

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


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        PathState state = PathState.Lift;
        outtake.initAutoSpecimen();
        intake.retractSlide();

        waitForStart();

        outtake.scoreSpecimen();
        intake.servoToNeutral();

        elapsedTime = new ElapsedTime();


        while (opModeIsActive() && !isStopRequested()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            switch (state){
                case Lift:
                    if (elapsedTime.milliseconds()>200){
                        follower.followPath(scorePreload);
                        elapsedTime= new ElapsedTime();
                        state=PathState.Start;
                    }
                    break;
                case Start:
                    if (!follower.isBusy() || elapsedTime.milliseconds()>1500){

                        elapsedTime = new ElapsedTime();
                        state = PathState.ToSub;
                        driveTrain.drive(1);

                    }
                    break;
                case ToSub:
                    if (elapsedTime!=null && elapsedTime.milliseconds()>0){
                        driveTrain.drive(0);
                        outtake.openClawAuto();
                        state = PathState.ScorePreload;
                        elapsedTime = new ElapsedTime();
                    }
                    break;
                case ScorePreload:
                    if (elapsedTime!=null && elapsedTime.milliseconds()>100){
                        follower.followPath(pushSample1);
                        outtake.closeClaw();
                        outtake.moveToPickUpFromWall();
                        state = PathState.Sample3;
                        elapsedTime = null;
                    }
                    break;

                case Sample3:
                    if (!follower.isBusy()){
                        outtake.openClaw();
                        follower.followPath(pickup1);
                        state= PathState.PickUpSpec;
                        elapsedTime = null;
                    }
                    break;
                case PickUpSpec:
                    if (!follower.isBusy()){
                        if (elapsedTime==null) {
                            outtake.closeClaw();
                            elapsedTime= new ElapsedTime();

                        } else if (elapsedTime.milliseconds()>150){
                            follower.followPath(score);
                            outtake.scoreSpecimen();
                            elapsedTime=null;
                            if(cycleCount <= 4) {
                                state = PathState.ScoreSpec3;
                            }
                            if(cycleCount > 4) {
                                state = PathState.Score;
                            }

                        }

                    }
                    break;
                case ScoreSpec3:
                    if (!follower.isBusy()){
                        if (elapsedTime==null) {
                            outtake.openClawAuto();
                            cycleCount++;
                            elapsedTime= new ElapsedTime();

                        } else if (elapsedTime.milliseconds()>0){
                            follower.followPath(towall);
                            outtake.closeClaw();
                            outtake.moveToPickUpFromWall();
                            elapsedTime = null;
                            state = PathState.PickUpSpec;

                        }
                    }
                    break;

//                case Score:
//                    if (!follower.isBusy()){
//                        if (elapsedTime==null) {
//                            //outtake.openClaw();
//                            elapsedTime= new ElapsedTime();
//
//                        } else if (elapsedTime.milliseconds()>150){
//                            follower.followPath(pickUp);
//                            //outtake.moveToPickUpFromWall();
//                            elapsedTime=null;
//                            state= PathState.End;
//                        }
//                    }
//                    break;
//                case End:
//                    if (!follower.isBusy()){
//                        //outtake.setTarget(0);
//                    }
//                    break;
            }


            outtake.update();
            intake.update();
            follower.update();
        }
    }

    protected void buildPaths(){

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoringPose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        pushSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoringPose),new Point(midPoint0), new Point(midPoint1), new Point(midPoint2), new Point(pushPose1)))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), pushPose1.getHeading())
                .addPath(new BezierLine(new Point(pushPose1),new Point(endPushPose1)))
                .setLinearHeadingInterpolation(pushPose1.getHeading(), endPushPose1.getHeading())
                .addPath(new BezierCurve(new Point(endPushPose1),new Point(pushPose1),new Point (pushPose2)))
                .setLinearHeadingInterpolation(endPushPose1.getHeading(), pushPose2.getHeading())
                .addPath(new BezierLine(new Point(pushPose2), new Point(endPushPose2)))
                .setLinearHeadingInterpolation(pushPose2.getHeading(), endPushPose2.getHeading())
                .addPath(new BezierCurve(new Point(endPushPose2),new Point(pushPose2),new Point (pushPose3)))
                .setLinearHeadingInterpolation(endPushPose2.getHeading(), pushPose3.getHeading())
                .addPath(new BezierLine(new Point(pushPose3), new Point(endPushPose3)))
                .setLinearHeadingInterpolation(pushPose3.getHeading(), endPushPose3.getHeading())
                .build();

//        pushSample2 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(endPushPose1),new Point(pushPose1),new Point (pushPose2)))
//                .setLinearHeadingInterpolation(endPushPose1.getHeading(), pushPose2.getHeading())
//                .addPath(new BezierLine(new Point(pushPose2), new Point(endPushPose2)))
//                .setLinearHeadingInterpolation(pushPose2.getHeading(), endPushPose2.getHeading())
//                .build();
//
//        pushSample3 = follower.pathBuilder()
//
//                .build();

        pickup1 = new Path(new BezierLine(new Point(endPushPose3), new Point(pickupPose)));
        pickup1.setLinearHeadingInterpolation(endPushPose3.getHeading(), pickupPose.getHeading());

        score = new Path(new BezierLine(new Point(pickupPose), new Point(scoringPose.getX(), scoringPose.getY()+2)));
        score.setLinearHeadingInterpolation(pickupPose.getHeading(), scoringPose.getHeading());

        towall = new Path(new BezierLine(new Point(scoringPose.getX(), scoringPose.getY()+2), new Point(pickupPose1)));
        towall.setLinearHeadingInterpolation(scoringPose.getHeading(), pickupPose1.getHeading());

        pickUp = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoringPose), new Point(pickupPose1), new Point(pickupPose)))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), pickupPose.getHeading())
                .build();




    }
}
