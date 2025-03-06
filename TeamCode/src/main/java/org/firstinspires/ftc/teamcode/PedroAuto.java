package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.bots.BotBot.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bots.BotBot;
import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.RollerIntakeBot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.sample.Sample;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Pedro Auto Samples", group = "Auto")
public class PedroAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private ElapsedTime initTimer;

    private boolean isReady = false;


    private ElapsedTime actiontime = new ElapsedTime();

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(7.591433278418451, 113.39703459637562, Math.toRadians(0));// starting position of robot
//    private final Pose scoreSpecimenPose = new Pose(12.7, 30, Math.toRadians(0));// position where specimen is scored on submersible, robot is aligned to submerisble with back facing it

    //    private final Pose sample1 = new Pose(35, 23,0); //these three poses are just behind the samples
//    private final Pose samplePivotPose = new Pose(35, 12.7,0); //pivot from one point to grab all 3 samples

    private final Pose samplePickupPose1 = new Pose(27.9, 134.7479406919275, Math.toRadians(-43));

    private final Pose samplePickupPose2 = new Pose(30.75, 107.70345963756178, Math.toRadians(55));

    private final Pose samplePickupPose3 = new Pose(29.8, 120.667215815486, Math.toRadians(55));
//    private final Pose sample3 = new Pose(35, 6,0);

//    private final Pose dropSamplePose = new Pose(28, 136, Math.toRadians(180));

    private final Pose loadSpecimenPose = new Pose(7.9, 23.6, 0);

    private final Pose scoreSamplePose = new Pose(12., 125.2, Math.toRadians(-45));

    private final Pose sampleCurveControlPoint = new Pose(21.8, 36.7, Math.toRadians(-36));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 46, Math.toRadians(90));

    /** coordinate to control bezier curve for parking, to go around the submersible must use bezier curve, this is mid point.*/
    private final Pose parkControl = new Pose (37, 25, 0);

    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry telemetryA;

    private Path scorePreload, park;

    private PathChain pickup1, pickup2, pickup3, dropoff, loadSpecimen, scoreSpecimen,scoreSample1,scoreSample2,scoreSample3;

    public void buildPaths(){

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSamplePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scoreSamplePose.getHeading());

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSamplePose), new Point (samplePickupPose1)))
                .setLinearHeadingInterpolation(scoreSamplePose.getHeading(), samplePickupPose1.getHeading())
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSamplePose), new Point(samplePickupPose2)))
                .setLinearHeadingInterpolation(scoreSamplePose.getHeading(), samplePickupPose2.getHeading())
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSamplePose), new Point(samplePickupPose3)))
                .setLinearHeadingInterpolation(scoreSamplePose.getHeading(), samplePickupPose3.getHeading())
                .build();
//        loadSpecimen = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scoreSamplePose), new Point(loadSpecimenPose)))
//                .setLinearHeadingInterpolation(scoreSamplePose.getHeading(), loadSpecimenPose.getHeading())
//                .build();

//        scoreSpecimen = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(loadSpecimenPose), new Point(scoreSpecimenPose)))
//                .setLinearHeadingInterpolation(loadSpecimenPose.getHeading(), scoreSpecimenPose.getHeading())
//                .build();

        park = new Path(new BezierCurve(new Point(scoreSamplePose), /* Control Point */ new Point(parkControl), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scoreSamplePose.getHeading(), parkPose.getHeading());

        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePickupPose1), new Point(scoreSamplePose)))
                .setLinearHeadingInterpolation(samplePickupPose1.getHeading(), scoreSamplePose.getHeading())
                .build();
        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePickupPose2), new Point(scoreSamplePose)))
                .setLinearHeadingInterpolation(samplePickupPose2.getHeading(), scoreSamplePose.getHeading())
                .build();
        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePickupPose3), new Point(scoreSamplePose)))
                .setLinearHeadingInterpolation(samplePickupPose3.getHeading(), scoreSamplePose.getHeading())
                .build();
    }

    protected AutonBot robot = new AutonBot(this);
//    private PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.isAuto = true;
//        robot.init(hardwareMap);
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        initTimer = new Timer();
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//        robot.currentState = FSMBot.gameState.DRIVE;
//        buildPaths();
//        opmodeTimer.resetTimer();
//        setPathState(0);
        waitForStart();

        robot.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        initTimer = new ElapsedTime();
        initTimer.reset();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        robot.currentState = FSMBot.gameState.DRIVE;
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(0);



        while (opModeIsActive()&&initTimer.milliseconds()>100) {
            telemetry.addData("pivot pos", robot.getPivotPosition());
            telemetry.addData("slide pos", robot.getSlidePosition());
            telemetry.addData("Pivot target", robot.pivotTarget);
            telemetry.addData("path state", pathState);
            telemetry.addData("follower busy", follower.isBusy());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Follower path", follower.getCurrentPath());
            telemetry.addData("state", robot.currentState);
            telemetry.addData("action time", actiontime.milliseconds());


            dashboard.getTelemetry();
            robot.updateStates();

            telemetry.update();
            follower.update();
            autonomousPathUpdate();
        }
    }
    public void autonomousPathUpdate()  {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload); /**not path chain, may be error */
                telemetry.addData("Current state", "State 0, Score Preload Pose");
//                    robot.sleep(100);
                setPathState(1);
                actiontime.reset();
                isReady = false;
                //Goes to bucket, in position to score preload and raises arm
                break;
            case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    if(isReady == false) {
                        if (actiontime.milliseconds() > 2000) {
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_2;
                        } else{
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
                        }
                        if (robot.getSlidePosition() > 660-25) {
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_3;
                            robot.outake(true);
                            isReady = true;
                            actiontime.reset();
                        }
                    }
                    else {
                        if(actiontime.milliseconds() > 1000) {
                            robot.slideRunToPosition(0);
                        }
                        if(actiontime.milliseconds() > 1000) {
                            follower.followPath(pickup1, true);
                            actiontime.reset();
                            isReady = false;
                            setPathState(2);
                        }
                    }

                }else{
                    actiontime.reset();
                }
                    /* Score Preload */
//                    robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
//                    //INSERT 3DOF CODE HERE TO SCORE SPECIMEN
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
////                         follower.followPath(grabPickup1,true);
//                    robot.triggerEvent(EVENT_PRELOAD_SCORED, 2);

                break;

            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    robot.pivotRunToPosition(0);
                    robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_2;
                    robot.intake(true);
                    if(robot.getPivotPosition() < 100) {
                        robot.slidePower = 0.6;
                        //extend slide out to intake
                        robot.slideRunToPosition(650);
                    }
                    if(robot.getSlidePosition() > 630 ){
                        robot.currentState = FSMBot.gameState.DRIVE;
                        robot.slidePower = 1;
                        //go to score
                            follower.followPath(scoreSample1, true);
                            isReady = false;
                            setPathState(3);
                    }
//                    //pickup first sample
//                    if(robot.getIsIntaked()) {
//                        robot.triggerEvent(EVENT_SAMPLE_1_PICKEDUP, 3);
//                        setPathState(EVENT_SAMPLE_1_PICKEDUP);
//                    }

                }
                break;


            case 3:
                if (!follower.isBusy()) {
                    if(isReady == false) {
                        if (actiontime.milliseconds() > 2000) {
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_2;
                        } else{
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
                        }
                        if (robot.getSlidePosition() > 660-25) {
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_3;
                            robot.outake(true);
                            isReady = true;
                            actiontime.reset();
                        }}
                    else {
                        if(actiontime.milliseconds() > 1000) {
                            robot.slideRunToPosition(0);
                        }
                        if(actiontime.milliseconds() > 1350) {
                            robot.pivotRunToPosition(0);
                            robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
                            follower.followPath(pickup2, true);
                            isReady = false;
                            setPathState(4);
                        }
                    }

                }else{
                    actiontime.reset();
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    if (robot.currentState != FSMBot.gameState.SUBMERSIBLE_INTAKE_2) {
                        robot.intake(true);
                        robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
                    }
                    else {
                        robot.slidePower = 0.6;
                        robot.intake(true);
                        //extend slide out to intake
                        robot.slideRunToPosition(650);
                    }
                    if(robot.getSlidePosition() > 630 || robot.getIsIntaked()) {
                        robot.currentState = FSMBot.gameState.DRIVE;
                        if(actiontime.milliseconds() > 2000){
                            robot.slidePower = 1;
                            //go to score
                            follower.followPath(scoreSample2, true);
                            isReady = false;
                            setPathState(5);
                        }
                    }
//                    //pickup first sample
//                    if(robot.getIsIntaked()) {
//                        robot.triggerEvent(EVENT_SAMPLE_1_PICKEDUP, 3);
//                        setPathState(EVENT_SAMPLE_1_PICKEDUP);
//                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if(isReady == false) {
                        if (actiontime.milliseconds() > 2000) {
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_2;
                        } else{
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
                        }
                        if (robot.getSlidePosition() > 660-25) {
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_3;
                            robot.outake(true);
                            isReady = true;
                            actiontime.reset();
                        }}
                    else {
                        if(actiontime.milliseconds() > 1000) {
                            robot.slideRunToPosition(0);
                        }
                        if(actiontime.milliseconds() > 1350) {
                            robot.pivotRunToPosition(0);
                            robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
                            follower.followPath(pickup3, true);
                            isReady = false;
                            setPathState(6);
                        }
                    }

                }else{
                    actiontime.reset();
                }
                    /* Score Preload */
//                    robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
//                    //INSERT 3DOF CODE HERE TO SCORE SPECIMEN
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
////                         follower.followPath(grabPickup1,true);
//                    robot.triggerEvent(EVENT_PRELOAD_SCORED, 2);

                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    if (robot.currentState != FSMBot.gameState.SUBMERSIBLE_INTAKE_2) {
                        robot.intake(true);

                        robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
                    }
                    else {
                        robot.slidePower = 0.6;
                        robot.intake(true);
                        //extend slide out to intake
                        robot.slideRunToPosition(400);
                    }
                    if(robot.getSlidePosition() > 390 || robot.getIsIntaked()) {
                        robot.slidePower = 1.0;
                        robot.currentState = FSMBot.gameState.DRIVE;
                        if(actiontime.milliseconds() > 2000){
                            //go to score
                            follower.followPath(scoreSample3, true);
                            isReady = false;
                            setPathState(-1); // end here
                        }
                }
//                    //pickup first sample
//                    if(robot.getIsIntaked()) {
//                        robot.triggerEvent(EVENT_SAMPLE_1_PICKEDUP, 3);
//                        setPathState(EVENT_SAMPLE_1_PICKEDUP);
//                    }
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    if(isReady == false) {
                        if (actiontime.milliseconds() > 1000) {
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_2;
                        } else{
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
                        }
                        if (robot.getSlidePosition() > 700-25) {
                            robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_3;
                            robot.outake(true);
                            isReady = true;
                            actiontime.reset();
                        }}
                    else {
                        if(actiontime.milliseconds() > 1000) {
                            robot.slideRunToPosition(0);
                        }
                        if(actiontime.milliseconds() > 1350) {
                            robot.pivotRunToPosition(0);
                            robot.currentState = FSMBot.gameState.DRIVE;
                            isReady = false;
                            setPathState(-1);
                        }
                    }

                }else{
                    actiontime.reset();
                }
                    /* Score Preload */
//                    robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
//                    //INSERT 3DOF CODE HERE TO SCORE SPECIMEN
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
////                         follower.followPath(grabPickup1,true);
//                    robot.triggerEvent(EVENT_PRELOAD_SCORED, 2);


                break;

            case 8:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(scoreSample3, true);
                    //release sample with claw
//                    robot.triggerEvent(EVENT_SAMPLE_3_SCORED , 8);
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()){
                    follower.followPath(park, true);
//                    robot.triggerEvent(EVENT_PARKED);
                    setPathState(-1);
                }
        }
    }
    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


}
