package org.firstinspires.ftc.masters.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.components.ITDCons;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outtake;
import org.firstinspires.ftc.masters.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.masters.pedroPathing.constants.LConstants;

//position are setup with pedro coordinate from blue side
//auto can be used for blue and red

@Autonomous(name="specimen")
public class Specimen extends LinearOpMode {

    Pose startPose = new Pose(10,65,0);
    Pose scoringPose = new Pose(38,65, 0);
    Pose midPoint = new Pose(25,38,0);
    Pose pickupPose = new Pose (10,31, 0);
    Pose pushPose1 = new Pose(60,22,0);
    Pose endPushPose1 = new Pose (15,22,0);
    Pose pushPose2 = new Pose(60,12,0);
    Pose endPushPose2 = new Pose(15,12,0);
    Pose pushPose3 = new Pose(60,8,0);
    Pose endPushPose3 = new Pose(15,8,0);

    Path scorePreload, pickup1, score, pickUp;
    PathChain pushSample1, pushSample2, pushSample3;

    enum PathState {Start,ScorePreload,Sample1,PushSample1, Sample2, PushSample2, Sample3, PushSample3, PickUpSpec, Score, End}

    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO: put the actual start pose

        ElapsedTime elapsedTime = null;

        Init init = new Init(hardwareMap);
        Outtake outtake = new Outtake(init, telemetry);
        Intake intake = new Intake(init, telemetry);


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        PathState state = PathState.Start;

        waitForStart();

        outtake.setTarget(ITDCons.SpecimenTarget);
        follower.followPath(scorePreload);

        while (opModeIsActive() && !isStopRequested()) {



            switch (state){
                case Start:
                    if (!follower.isBusy()){
                        outtake.openClaw();
                        elapsedTime = new ElapsedTime();
                        state = PathState.ScorePreload;
                        outtake.setStatus(Outtake.Status.ScoreSpecimen);

                    }
                    break;
                case ScorePreload:
                    if (elapsedTime!=null && elapsedTime.milliseconds()>150){
                        follower.followPath(pushSample1);
                        state = PathState.Sample1;
                        outtake.moveToPickUpFromWall();
                        elapsedTime = null;
                    }
                    break;
                case Sample1:
                    if (!follower.isBusy()){
                        follower.followPath(pushSample2);
                        state= PathState.Sample2;
                    }
                    break;
                case Sample2:
                    if (!follower.isBusy()){
                        follower.followPath(pushSample3);
                        state= PathState.Sample3;
                    }
                    break;
                case Sample3:
                    if (!follower.isBusy()){
                        follower.followPath(pickup1);
                        state= PathState.PickUpSpec;
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
                            state= PathState.Score;
                        }
                    }
                    break;
                case Score:
                    if (!follower.isBusy()){
                        if (elapsedTime==null) {
                            outtake.openClaw();
                            elapsedTime= new ElapsedTime();

                        } else if (elapsedTime.milliseconds()>150){
                            follower.followPath(pickUp);
                            outtake.moveToPickUpFromWall();
                            elapsedTime=null;
                            state= PathState.End;
                        }
                    }
                    break;
                case End:
                    if (!follower.isBusy()){
                        outtake.setTarget(0);
                    }
                    break;
            }


            outtake.updateOuttake();
            follower.update();
        }
    }

    protected void buildPaths(){

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoringPose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        pushSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoringPose), new Point(midPoint), new Point(pushPose1)))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), pushPose1.getHeading())
                .addPath(new BezierLine(new Point(pushPose1),new Point(endPushPose1)))
                .build();

        pushSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(endPushPose1),new Point(pushPose1),new Point (pushPose2)))
                .addPath(new BezierLine(new Point(pushPose2), new Point(endPushPose2)))
                .build();

        pushSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(endPushPose2),new Point(pushPose2),new Point (pushPose3)))
                .addPath(new BezierLine(new Point(pushPose3), new Point(endPushPose3)))
                .build();

        pickup1 = new Path(new BezierLine(new Point(endPushPose3), new Point(pickupPose)));

        score = new Path(new BezierLine(new Point(pickupPose), new Point(scoringPose)));

        pickUp = new Path(new BezierLine(new Point(scoringPose), new Point(pickupPose)));





    }
}
