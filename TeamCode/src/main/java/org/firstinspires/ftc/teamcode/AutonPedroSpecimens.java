package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.bots.AutomationBot;
import org.firstinspires.ftc.teamcode.bots.HangBot;
import org.firstinspires.ftc.teamcode.bots.LimelightBot;

import org.firstinspires.ftc.teamcode.bots.OdometryBot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "AutonPedro", group = "Autos")
public class AutonPedroSpecimens extends LinearOpMode{
    protected boolean isBlue = false;
    protected HangBot robot = new HangBot(this);
    private int lastMotorPosition;

    private double startingTime;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;


    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9.45, 72, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(40, 72, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */

    private final Pose push1Pose = new Pose(60, 38, Math.toRadians(0));

    private final Pose push1Control = new Pose(33.4,35, Math.toRadians(0));

    private final Pose endPush1Pose =  new Pose(20,32, Math.toRadians(0));

    private final Pose push1Control2 = new Pose(60,32, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */

    private final Pose push2ControlPose = new Pose(100, 23.5, Math.toRadians(0));

    private final Pose push2Pose = new Pose(20, 20.1, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */

    private final Pose push3ControlPose = new Pose(100, 18, Math.toRadians(0));

    private final Pose push3Pose = new Pose(20, 14, Math.toRadians(0));

    /** Pickup Samples form Human player area */

    private final Pose pickupPoseControl = new Pose(47.5,22.1, Math.toRadians(270));

    private final Pose pickupReadyPose = new Pose(15.1,48, Math.toRadians(270));
    private final Pose pickupPose = new Pose(15.1,40, Math.toRadians(270));

    private final Pose scoreControlPose = new Pose(15,63, Math.toRadians(0));
    private final Pose obsScorePose = new Pose(40, 80, Math.toRadians(0));


    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain rungToPush, pushSpec1, pushSpec2, pushSpec3, specReadyToScore, readyToScore, scoreFromObservation, moveIn;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        rungToPush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(push1Control), new Point(push1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), push1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSpec1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push1Pose), new Point(push1Control2), new Point(endPush1Pose)))
                .setLinearHeadingInterpolation(push1Pose.getHeading(), endPush1Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSpec2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(endPush1Pose), new Point(push2ControlPose), new Point(push2Pose)))
                .setLinearHeadingInterpolation(endPush1Pose.getHeading(), push2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSpec3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push2Pose), new Point(push3ControlPose), new Point(push3Pose)))
                .setLinearHeadingInterpolation(push2Pose.getHeading(), push3Pose.getHeading())
                .build();



        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        specReadyToScore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push2Pose), new Point(pickupPoseControl), new Point(pickupPose)))
                .setLinearHeadingInterpolation(push2Pose.getHeading(), pickupPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreFromObservation = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose), new Point(obsScorePose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), obsScorePose.getHeading())
                .build();

        readyToScore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),new Point(scoreControlPose), new Point(pickupReadyPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
                .build();

        moveIn = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupReadyPose), new Point(pickupPose)))
                .setLinearHeadingInterpolation(pickupReadyPose.getHeading(), pickupPose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        //park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        //park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //robot.init(hardwareMap);
                //robot.scoreSpecimen(true);
                //robot.pinchControl(false,true);
                follower.followPath(scorePreload);
                robot.readySpecimenPos(true, true);
                setPathState(1);
                break;
            case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    robot.slideTarget = lastMotorPosition - 500;
                    robot.readySpecimenPos(true, true);
                    /*robot.scoreSpecimen(true);
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(pathTimer.getElapsedTimeSeconds() > startingTime + 2) {
                        follower.followPath(rungToPush, true);
                        setPathState(2);
                    }
                }
                else {
                    startingTime =  pathTimer.getElapsedTimeSeconds();
                    lastMotorPosition = robot.slideTarget;
                }

                break;
            case 2:
                robot.openPinch();
                robot.slideTarget = 0;
                robot.pivotTarget = 60;
                /*robot.pivotTo(0,0.5);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.getPose().getX() > (push1Pose.getX() - 1) && follower.getPose().getY() > (push1Pose.getY() - 1)) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pushSpec1,false);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushSpec2,false);
                    setPathState(5);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pushSpec3,false);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    robot.pivotTarget = 0;
                    /* Score Sample */
                    /*robot.pinchControl(false,true);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(specReadyToScore,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {
                    /* Grab Sample */
                    /*robot.pinchControl(false,true);
                    /*robot.pickup(isBlue, false, true, telemetry);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(0.5);
                    follower.followPath(moveIn, true);

                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(follower.getPose().getX() > (pickupPose.getX() - 0.2) && follower.getPose().getY() < (pickupPose.getY() + 0.2)) {
                    robot.closePinch();
                    /* Grab Sample */
                    /*robot.pinchControl(false,true);
                    /*robot.pickup(isBlue, false, true, telemetry);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds() > startingTime + 2) {
                        follower.followPath(scoreFromObservation, true);
                        follower.setMaxPower(1);
                        robot.pivotTarget = 300;
                        setPathState(-1);
                    }
                }
                else {
                    startingTime =  pathTimer.getElapsedTimeSeconds();
                    lastMotorPosition = robot.slideTarget;
                }
                break;
            case 8:
                robot.readySpecimenPos(true,true);
                startingTime = pathTimer.getElapsedTime();
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {

                    robot.slideTarget = lastMotorPosition + 500;
                    robot.slideMotor.setPower(0.2);

                    /* Score Sample */
                    /*robot.scoreSpecimen(true);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    if(pathTimer.getElapsedTimeSeconds() > startingTime + 1) {
                        follower.followPath(readyToScore, true);
                        setPathState(9);
                        robot.openPinch();
                        robot.slideMotor.setPower(0.6);
                        robot.slideTarget = 0;
                        robot.pivotTarget = 0;
                    }
                } else {
                    lastMotorPosition = robot.slideTarget;
                }
                break;


            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void runOpMode() throws InterruptedException {
        robot.isAuto = true;
        robot.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(0);
        waitForStart();

        while (isStarted() == true) {

            // These loop the movements of the robot
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }

    }
}
