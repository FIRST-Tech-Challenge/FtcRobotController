package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Pedro Auto Specimens", group = "Auto")
public class PedroAutoSpecimens extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private boolean isReady = false;

    private ElapsedTime actiontime = new ElapsedTime();

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(8.065897858319605, 66.1878088962109, Math.toRadians(0));// starting position of robot
//    private final Pose scoreSpecimenPose = new Pose(12.7, 30, Math.toRadians(0));// position where specimen is scored on submersible, robot is aligned to submerisble with back facing it

    //    private final Pose sample1 = new Pose(35, 23,0); //these three poses are just behind the samples
    private final Pose forwardScorePose = new Pose(39.38056013179572, 66.1878088962109,0); //pivot from one point to grab all 3 samples

    private final Pose pickupSample1PoseControl = new Pose(35.34761120263591,45.54859967051071, 0 );

    private final Pose pickupSample1Pose = new Pose(41.990115321252055,37.95716639209226,Math.toRadians(-60));

    private final Pose dropoffSample1Pose = new Pose(28.70510708401977,33.68698517298188, Math.toRadians(-160));

    private final Pose pickupSample2Pose = new Pose(43.650741350906095,28.467874794069186,Math.toRadians(-85));

    private final Pose dropoffSample2Pose = new Pose(31.55189456342669,24.67215815485996, Math.toRadians(-160));

    private final Pose pickupSample3Pose =  new Pose(41.990115321252055,41.990115321252055,Math.toRadians(-85));

    private final Pose pickupSpecimenPose = new Pose(7.116968698517298,30.365733113673798, Math.toRadians(180));

    private final Pose pickupSpecimenControlPose = new Pose(48.39538714991763,34.87314662273476, Math.toRadians(180));

    private final Pose dropoffSpecimenPose = new Pose(42.701812191103784,65.95057660626028, Math.toRadians(180));


    /** coordinate to control bezier curve for parking, to go around the submersible must use bezier curve, this is mid point.*/

    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry telemetryA;


    private Path scorePreload, park;

    private PathChain pickupSample1, dropoffSample1, pickupSample2, dropoffSample2, pickupSample3, samplesToSpecimen, scoreSpecimen, pickupSpecimen;

    public void buildPaths(){
        scorePreload = new Path(new BezierLine (new Point(startPose), new Point(forwardScorePose)));

        pickupSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(forwardScorePose), new Point (pickupSample1Pose)))
                .setLinearHeadingInterpolation(forwardScorePose.getHeading(), pickupSample1Pose.getHeading())
                .build();
        dropoffSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSample1Pose),  new Point(pickupSample1PoseControl), new Point(dropoffSample1Pose)))
                .setLinearHeadingInterpolation(pickupSample1Pose.getHeading(), dropoffSample1Pose.getHeading())
                .build();
        pickupSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forwardScorePose), new Point (pickupSample1Pose)))
                .setLinearHeadingInterpolation(forwardScorePose.getHeading(), pickupSample1Pose.getHeading())
                .build();
        dropoffSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSample2Pose), new Point(dropoffSample2Pose)))
                .setLinearHeadingInterpolation(pickupSample2Pose.getHeading(), dropoffSample2Pose.getHeading())
                .build();
        pickupSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffSample2Pose), new Point (pickupSample3Pose)))
                .setLinearHeadingInterpolation(dropoffSample2Pose.getHeading(), pickupSample3Pose.getHeading())
                .build();
        samplesToSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSample3Pose), new Point(pickupSpecimenPose)))
                .setLinearHeadingInterpolation(pickupSample3Pose.getHeading(), pickupSpecimenPose.getHeading())
                .build();
        scoreSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSpecimenPose), new Point(dropoffSpecimenPose)))
                .setLinearHeadingInterpolation(pickupSpecimenPose.getHeading(), dropoffSpecimenPose.getHeading())
                .build();
        pickupSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffSpecimenPose), new Point(pickupSpecimenPose)))
                .setLinearHeadingInterpolation(dropoffSpecimenPose.getHeading(), pickupSpecimenPose.getHeading())
                .build();
    }

    protected AutonBot robot = new AutonBot(this);
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
        robot.currentState = FSMBot.gameState.DRIVE;
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(0);
        waitForStart();
        while (opModeIsActive()) {
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
                robot.currentState = FSMBot.gameState.SPECIMEN_SCORING_HIGH_DRIVE;
                follower.followPath(scorePreload);
                setPathState(1);
            case 1:
                //score specimenPreload
                if (!follower.isBusy()) {
                    if(actiontime.milliseconds() < 1000)
                            robot.currentState = FSMBot.gameState.SUBMERSIBLE_INTAKE_2;
                            robot.slideRunToPosition(400);
                            follower.followPath(pickupSample1);
                            isReady = false;
                            setPathState(2);
                    } else {
                    actiontime.reset();
                }
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(dropoffSample1);
                    setPathState(3);
                }

        }
    }
    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


}
