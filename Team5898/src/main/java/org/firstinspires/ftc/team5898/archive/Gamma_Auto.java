package org.firstinspires.ftc.team5898.archive;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team5898.RobotHardware;
import org.firstinspires.ftc.team5898.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.team5898.pedroPathing.constants.LConstants;

@Autonomous(name = "Gamma Auto w/Pedro", group = "Examples")
public class Gamma_Auto extends OpMode {

    private Follower follower;
    private RobotHardware robot;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 105, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(19, 122.5, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(34, 121.5, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(34, 131, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(59, 100, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(67, 127, Math.toRadians(330));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, scorePickup1, scorePickup2;

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
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                //outtake flips and the slides rise up
                robot.rightOuttake.setPosition(1);
                robot.leftOuttake.setPosition(0);
                //add code to make slides go up and hold here
                // Set a target encoder position for the lift (e.g., 500 ticks)
                robot.setLiftPosition(3050, 1);

                if (opmodeTimer.getElapsedTimeSeconds() > 1.5)
                {
                    follower.followPath(scorePreload);
                    setPathState(1);
                    opmodeTimer.resetTimer();
                }

                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    if (opmodeTimer.getElapsedTimeSeconds() > 2 && opmodeTimer.getElapsedTimeSeconds() < 2.5)
                    {
                        //release sample, bring outtake back in
                        robot.claw.setPosition(robot.CLAW_OPEN);
                        robot.rightOuttake.setPosition(0);
                        robot.leftOuttake.setPosition(1);
                    }
                    else if (opmodeTimer.getElapsedTimeSeconds() > 3.5)
                    {
                        // add code to make slides go back down and chill here
                        robot.setLiftPosition(20, .7);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup1,true);
                        setPathState(2);
                        opmodeTimer.resetTimer();
                        robot.grabber.setPosition(robot.GRABBER_OPEN); //open
                        robot.wrist.setPosition(robot.WRIST_HOVER);
                    }

                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 1.5 && opmodeTimer.getElapsedTimeSeconds() < 3) {
                        /* Grab Sample */
                        robot.wrist.setPosition(robot.WRIST_GRAB);
                        robot.grabber.setPosition(0.2);
                    }

                    if (opmodeTimer.getElapsedTimeSeconds() > 3 && opmodeTimer.getElapsedTimeSeconds() < 4) {
                        robot.wrist.setPosition(robot.WRIST_BACK);
                        robot.leftIntake.setPosition(robot.INTAKE_IN_LEFT);
                        robot.rightIntake.setPosition(robot.INTAKE_IN_RIGHT);
                    }
                    if (opmodeTimer.getElapsedTimeSeconds() > 4 && opmodeTimer.getElapsedTimeSeconds() < 5) {
                        //transition sample to outtake claw and wait for input
                        robot.claw.setPosition(robot.CLAW_CLOSE);
                    }
                    if (opmodeTimer.getElapsedTimeSeconds() > 5 && opmodeTimer.getElapsedTimeSeconds() < 6) {
                        robot.grabber.setPosition(robot.GRABBER_OPEN);
                        //robot.wrist.setPosition(robot.WRIST_MID);

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup1,true);
                        setPathState(3);
                        opmodeTimer.resetTimer();
                    }

                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    robot.setLiftPosition(3050, 1);
                    if (opmodeTimer.getElapsedTimeSeconds() > 3 && opmodeTimer.getElapsedTimeSeconds() < 4){
                        //release sample, bring outtake back in
                        robot.rightOuttake.setPosition(1);
                        robot.leftOuttake.setPosition(0);
                    }
                    else if (opmodeTimer.getElapsedTimeSeconds() > 4 && opmodeTimer.getElapsedTimeSeconds() < 6){
                        robot.claw.setPosition(robot.CLAW_OPEN);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup2,true);
                        setPathState(4);
                        opmodeTimer.resetTimer();
                        robot.grabber.setPosition(robot.GRABBER_OPEN);
                        robot.wrist.setPosition(robot.WRIST_HOVER);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    robot.setLiftPosition(20, 0.7);
                    robot.rightOuttake.setPosition(0);
                    robot.leftOuttake.setPosition(1);
                    if (opmodeTimer.getElapsedTimeSeconds() > 1.5 && opmodeTimer.getElapsedTimeSeconds() < 3) {
                        /* Grab Sample */
                        robot.wrist.setPosition(robot.WRIST_GRAB);
                        robot.grabber.setPosition(0.2);
                    }
                    if (opmodeTimer.getElapsedTimeSeconds() > 3 && opmodeTimer.getElapsedTimeSeconds() < 4) {
                        robot.wrist.setPosition(robot.WRIST_BACK);
                        robot.leftIntake.setPosition(robot.INTAKE_IN_LEFT);
                        robot.rightIntake.setPosition(robot.INTAKE_IN_RIGHT);
                    }
                    if (opmodeTimer.getElapsedTimeSeconds() > 4 && opmodeTimer.getElapsedTimeSeconds() < 5) {
                        //transition sample to outtake claw and wait for input
                        robot.claw.setPosition(robot.CLAW_CLOSE);
                    }
                    if (opmodeTimer.getElapsedTimeSeconds() > 5 && opmodeTimer.getElapsedTimeSeconds() < 6) {
                        robot.grabber.setPosition(robot.GRABBER_OPEN);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup2,true);
                        setPathState(5);
                        opmodeTimer.resetTimer();
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    robot.setLiftPosition(3050, 1);
                    if (opmodeTimer.getElapsedTimeSeconds() > 3 && opmodeTimer.getElapsedTimeSeconds() < 4){
                        //release sample, bring outtake back in
                        robot.rightOuttake.setPosition(1);
                        robot.leftOuttake.setPosition(0);
                    }
                    else if (opmodeTimer.getElapsedTimeSeconds() > 4 && opmodeTimer.getElapsedTimeSeconds() < 6){
                        robot.claw.setPosition(robot.CLAW_OPEN);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(park,true);
                        setPathState(6);
                        opmodeTimer.resetTimer();
                        robot.grabber.setPosition(robot.GRABBER_OPEN);
                        robot.wrist.setPosition(robot.WRIST_HOVER);
                    }
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    robot.setLiftPosition(20, 0.7);
                    robot.rightOuttake.setPosition(0);
                    robot.leftOuttake.setPosition(1);
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }

        telemetry.addData("State: ", pathState);
        telemetry.addData("OpMode Timer: ", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Is Follower Busy: ", follower.isBusy());
        telemetry.update();
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

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

    /** This method is called once at the init of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = new RobotHardware(hardwareMap);
        robot.init();  // Initialize all hardware

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        robot.rightIntake.setPosition(robot.INTAKE_IN_RIGHT);
        robot.leftIntake.setPosition(robot.INTAKE_IN_LEFT);
        robot.wrist.setPosition(robot.WRIST_NEUTRAL);
        robot.grabber.setPosition(robot.GRABBER_CLOSE); //grabber closed
        robot.rightOuttake.setPosition(0); //waiting to grab
        robot.leftOuttake.setPosition(1); //waiting to grab
        robot.claw.setPosition(robot.CLAW_CLOSE); // claw resting open
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.**/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

