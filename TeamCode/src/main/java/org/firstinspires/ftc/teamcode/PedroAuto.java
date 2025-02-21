package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.bots.BotBot.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bots.BotBot;
import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.RollerIntakeBot;
import org.firstinspires.ftc.teamcode.bots.PedroPathingSpecimenBot;
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

@Autonomous(name = "Pedro Auto", group = "Auto")
public class PedroAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(7.591433278418451, 114.1, Math.toRadians(0));// starting position of robot
//    private final Pose scoreSpecimenPose = new Pose(12.7, 30, Math.toRadians(0));// position where specimen is scored on submersible, robot is aligned to submerisble with back facing it

    //    private final Pose sample1 = new Pose(35, 23,0); //these three poses are just behind the samples
    private final Pose samplePivotPose = new Pose(35, 12.7,0); //pivot from one point to grab all 3 samples

    private final Pose SamplePickupPose1 = new Pose(28, 124.30971993410215, Math.toRadians(30));
    private final Pose samplePickupPoseSweep1 = new Pose(26,124.30971993410215, Math.toRadians(0));

    private final Pose SamplePickupPose2 = new Pose(28, 132.61285008237232, Math.toRadians(-30));
    private final Pose samplePickupPoseSweep2 = new Pose(26,132.61285008237232, Math.toRadians(0));

    private final Pose SamplePickupPose3 = new Pose(28, 1374.35749588138387, Math.toRadians(0));
    private final Pose samplePickupPoseSweep3 = new Pose(26, 134.35749588138387, Math.toRadians(30));
//    private final Pose sample3 = new Pose(35, 6,0);

//    private final Pose dropSamplePose = new Pose(28, 136, Math.toRadians(180));

    private final Pose loadSpecimenPose = new Pose(7.9, 23.6, 0);

    private final Pose scoreSamplePose = new Pose(13.28500823723229, 130.24052718286654, -45);

    private final Pose sampleCurveControlPoint = new Pose(21.8, 36.7, Math.toRadians(-36));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 46, Math.toRadians(90));

    /** coordinate to control bezier curve for parking, to go around the submersible must use bezier curve, this is mid point.*/
    private final Pose parkControl = new Pose (37, 25, 0);
    
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry telemetryA;



    private Path scorePreload, park;

    private PathChain pickup1, pickupSweep1, pickup2, pickupSweep2, pickup3, pickupSweep3, dropoff, loadSpecimen, scoreSpecimen,scoreSample1,scoreSample2,scoreSample3;

    public void buildPaths(){
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSamplePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scoreSamplePose.getHeading());

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSamplePose), new Point (samplePickupPoseSweep1)))
                .setLinearHeadingInterpolation(scoreSamplePose.getHeading(), samplePickupPoseSweep1.getHeading())
                .build();
        pickupSweep1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePickupPoseSweep1), new Point (SamplePickupPose1)))
                .setLinearHeadingInterpolation(samplePickupPoseSweep1.getHeading(), SamplePickupPose1.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSamplePose), new Point(samplePickupPoseSweep2)))
                .setLinearHeadingInterpolation(scoreSamplePose.getHeading(), samplePickupPoseSweep2.getHeading())
                .build();
        pickupSweep2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePickupPoseSweep2), new Point (SamplePickupPose2)))
                .setLinearHeadingInterpolation(samplePickupPoseSweep2.getHeading(), SamplePickupPose2.getHeading())
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSamplePose), new Point(samplePickupPoseSweep3)))
                .setLinearHeadingInterpolation(scoreSamplePose.getHeading(), samplePickupPoseSweep3.getHeading())
                .build();
        pickupSweep3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePickupPoseSweep3), new Point (SamplePickupPose3)))
                .setLinearHeadingInterpolation(samplePickupPoseSweep3.getHeading(), SamplePickupPose3.getHeading())
                .build();
        dropoff = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePivotPose), new Point(scoreSamplePose)))
                .setLinearHeadingInterpolation(samplePivotPose.getHeading(), scoreSamplePose.getHeading())
                .build();
//
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
                .addPath(new BezierLine(new Point(SamplePickupPose1), new Point(scoreSamplePose)))
                .setLinearHeadingInterpolation(SamplePickupPose1.getHeading(), scoreSamplePose.getHeading())
                .build();
        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SamplePickupPose2), new Point(scoreSamplePose)))
                .setLinearHeadingInterpolation(SamplePickupPose2.getHeading(), scoreSamplePose.getHeading())
                .build();
        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SamplePickupPose3), new Point(scoreSamplePose)))
                .setLinearHeadingInterpolation(SamplePickupPose3.getHeading(), scoreSamplePose.getHeading())
                .build();
    }

    protected FSMBot robot = new FSMBot(this);
//    private PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.isAuto = true;
        robot.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        robot.currentState = FSMBot.gameState.PRE_DRIVE;
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(0);
        waitForStart();
        while (opModeIsActive()) {


            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Follower path", follower.getCurrentPath());
            dashboard.getTelemetry();
            
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
                setPathState(1);
                //Goes to submersible, in position to score preload
                break;
            case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()||pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pickup1);
                    /* Score Preload */
//                    robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
//                    //INSERT 3DOF CODE HERE TO SCORE SPECIMEN
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
////                         follower.followPath(grabPickup1,true);
//                    robot.triggerEvent(EVENT_PRELOAD_SCORED, 2);
                    setPathState(2);

                }
                break;

            case 2:

                if (!follower.isBusy()) {
                    follower.followPath(pickupSweep1, true);
//                    robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
//                    //pickup first sample
//                    if(robot.getIsIntaked()) {
//                        robot.triggerEvent(EVENT_SAMPLE_1_PICKEDUP, 3);
//                        setPathState(EVENT_SAMPLE_1_PICKEDUP);
//                    }
                    setPathState(3);
                }
                break;

            case 3:

//                if (!follower.isBusy()) {
//                    follower.followPath(pickup1, true);
//                    robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
//                    //pickup first sample
//                    if(robot.getIsIntaked()) {
//                        robot.triggerEvent(EVENT_SAMPLE_1_PICKEDUP, 3);
//                        setPathState(EVENT_SAMPLE_1_PICKEDUP);
//                    }
                    setPathState(4);
                //}
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(scoreSample1, true);

                    //release sample with claw
//                    robot.triggerEvent(EVENT_SAMPLE_1_SCORED , 4);
                    setPathState(5);
                }
                break;
            case 5:

                if (!follower.isBusy()) {
                    follower.followPath(pickup2, true);
//                    robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
//                    //pickup first sample
//                    if(robot.getIsIntaked()) {
//                        robot.triggerEvent(EVENT_SAMPLE_1_PICKEDUP, 3);
//                        setPathState(EVENT_SAMPLE_1_PICKEDUP);
//                    }
                    setPathState(6);
                }
                break;

            case 6:

                if (!follower.isBusy()) {
                    follower.followPath(pickupSweep2, true);
                    //pickup first sample
//                    robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
//                    if(robot.getIsIntaked()) {
//                        robot.triggerEvent(EVENT_SAMPLE_2_PICKEDUP, 5);
                        setPathState(7);
//                    }
                }
                break;

            case 7:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(scoreSample2, true);
//                    robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
//                    //release sample with claw
//                    robot.triggerEvent(EVENT_SAMPLE_2_SCORED , 6);
                    setPathState(8);
                }
                break;
            case 8:

                if (!follower.isBusy()) {
                    follower.followPath(pickup3, true);
//                    robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_1;
//                    //pickup first sample
//                    if(robot.getIsIntaked()) {
//                        robot.triggerEvent(EVENT_SAMPLE_1_PICKEDUP, 3);
//                        setPathState(EVENT_SAMPLE_1_PICKEDUP);
//                    }
                    setPathState(9);
                }
                break;

            case 10:

                if (!follower.isBusy()) {
                    follower.followPath(pickupSweep3, true);
                    //pickup first sample
                    robot.triggerEvent(EVENT_SAMPLE_3_PICKEDUP, 7);
                    setPathState(EVENT_SAMPLE_3_PICKEDUP);
                }
                break;

            case 11:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(scoreSample3, true);
                    //release sample with claw
//                    robot.triggerEvent(EVENT_SAMPLE_3_SCORED , 8);
                    setPathState(8);
                }
                break;

            case 12:
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
