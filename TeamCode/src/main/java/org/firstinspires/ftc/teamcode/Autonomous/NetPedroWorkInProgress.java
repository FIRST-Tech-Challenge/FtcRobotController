package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TwoFishDelivery;
import org.firstinspires.ftc.teamcode.TwoFishIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Net Pedro WIP", group = "Autonomous")
public class NetPedroWorkInProgress extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    ElapsedTime deliveryTimer = new ElapsedTime();

    TwoFishDelivery delivery = null;
    TwoFishIntake intake = null;
    DistanceSensor rearDistanceSens = null;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;


    private ElapsedTime auxilariesTimer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();

    private boolean isTransferring = false;
    private boolean isDelivering = false;
    private boolean isIntaking = false;
    boolean intakeIsPrepped = false;
    boolean transferIsPrepped = false;
    boolean deliveryIsPrepped = false;

    boolean distanceSensorDisabled = false;
    double distanceSensorX = 0.0;

    boolean isSamplePosessed = false;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Point startPoint = new Point(8, 110);

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Point scorePoint = new Point(14, 129);

    /** Lowest (First) Sample from the Spike Mark */
    private final Point pickup1Point = new Point(16, 126);

    /** Middle (Second) Sample from the Spike Mark */
    private final Point pickup2Point = new Point(16, 129);

    /** Highest (Third) Sample from the Spike Mark */
    private final Point pickup3Point = new Point(32, 122.5);

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Point parkPoint = new Point(60, 96);



    /* These are our Paths and PathChains that we will define in buildPaths() */

    public static PathChain scorePreload, intake1, score1, intake2, score2, intake3, score3, park;


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

        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPoint,
                                new Point(startPoint.getX() + 2, startPoint.getY())
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        new BezierCurve(
                                new Point(startPoint.getX() + 2, startPoint.getY()),
                                new Point(22.000, 118.000), //control point
                                scorePoint
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePoint,
                                pickup1Point
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                pickup1Point,
                                scorePoint
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePoint,
                                pickup2Point
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                pickup2Point,
                                scorePoint
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePoint,
                                pickup3Point
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(80))
                .build();

        score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                pickup3Point,
                                scorePoint
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-45))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePoint,
                                new Point(24.000, 100.000),
                                new Point(parkPoint.getX(), 130.000),
                                parkPoint
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .build();
    }





    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                prepDeliverSample();
                follower.followPath(scorePreload);
                if(sleepTimer.seconds() > 2)
                setPathState(1);
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
                    deliverSample();
                    if(!isDelivering) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intake1,true);
//                        if(sleepTimer.seconds() > 6) {
                            prepTransferSample();
                            setPathState(2);
//                        }
                    }
                }
                break;
            case 2:
//                if(sleepTimer.seconds() > 7)
                if(pathTimer.getElapsedTimeSeconds() > 0.5 && !isSamplePosessed) {
                    prepIntakeSample(1000);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the intake1's position */
//                if(sleepTimer.seconds() > 9)
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    prepTransferSample();
                    intakeSample();
                    if(!isIntaking) {
                        transferSample();
                        if (!isTransferring) {
                            prepDeliverSample();
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                            follower.followPath(score1, true);
                            setPathState(3);
                        }
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the score1's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    deliverSample();
                    if(!isDelivering) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(intake2,true);
//                        if(sleepTimer.seconds() > 6) {
                        prepTransferSample();
                        setPathState(4);
//                        }
                    }
                }
                break;
            case 4:

                if(pathTimer.getElapsedTimeSeconds() > 0.5 && !isSamplePosessed) {
                    prepIntakeSample(1000);
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    intakeSample();
                    prepTransferSample();
                    if(!isIntaking) {
                        transferSample();
                        if (!isTransferring) {
                            prepDeliverSample();
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                            follower.followPath(score2, true);
                            setPathState(5);
                        }
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    deliverSample();
                    if(!isDelivering) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(intake3,true);
//                        if(sleepTimer.seconds() > 6) {
                        prepTransferSample();
                        setPathState(6);
//                        }
                    }
                }
                break;
            case 6:

                if(pathTimer.getElapsedTimeSeconds() > 0.5 && !isSamplePosessed) {
                    prepIntakeSample(500);
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    intakeSample();
                    prepTransferSample();
                    if(!isIntaking) {
                        transferSample();
                        if (!isTransferring) {
                            prepDeliverSample();
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                            follower.followPath(score3, true);
                            setPathState(7);
                        }
                    }
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    deliverSample();
                    if(!isDelivering) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                        follower.followPath(park, false);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    private void prepDeliverSample(){
        if(!deliveryIsPrepped) {
            deliveryIsPrepped = true;
            delivery.toSampleHeight();
            delivery.setSlidesPower(0.75);
            delivery.setWrist(delivery.wristDownPosition);
            delivery.setPitch(delivery.pitchUpPosition);
        }
    }

    public void deliverSample(){
        if(!isDelivering && delivery.slideTarget != delivery.specHeight){
            auxilariesTimer.reset();
        }
        isDelivering = true;
        if(auxilariesTimer.seconds() < 0.5 && delivery.getPitch() != delivery.pitchSampleScorePosition){
            delivery.setPitch(delivery.pitchSampleScorePosition);
        }
        if(auxilariesTimer.seconds() > 0.5 && auxilariesTimer.seconds() < 0.75 && delivery.clawClosed){
            delivery.clawOpen();
        }
        if(auxilariesTimer.seconds() > 0.75 && auxilariesTimer.seconds() < 0.8 && delivery.getPitch() != delivery.pitchUpPosition){
            delivery.setPitch(delivery.pitchUpPosition);
        }
        if(auxilariesTimer.seconds() > 1){
            delivery.toSpecHeight();
            delivery.setSlidesPower(0.75);
            isSamplePosessed = false;
            isDelivering = false;
            deliveryIsPrepped = false;
        }
    }


    public void prepTransferSample() {
        if(!transferIsPrepped) {
            transferIsPrepped = true;
            delivery.toSpecHeight();
            delivery.setSlidesPower(0.75);
            delivery.setPitch(delivery.pitchTransferPosition);
            delivery.setWrist(delivery.wristUpPosition);
            delivery.clawOpen();
        }
    }

    public void prepIntakeSample(int targetLength) {
        if(!intakeIsPrepped) {
            intakeIsPrepped = true;
            delivery.toSpecHeight();
            delivery.setSlidesPower(0.75);
            intake.setSlidesTargetPosition(targetLength);
            intake.setSlidesPower(0.8);
            delivery.clawOpen();
        }
    }

    public void intakeSample(){
        if(!isIntaking && !isSamplePosessed){
            auxilariesTimer.reset();
        }
        if(!isSamplePosessed) isIntaking = true;
        if(auxilariesTimer.seconds() < 0.5 && !(intake.getPitch() == intake.downPitch) && !isSamplePosessed){
            intake.pitchUp();
            intake.setIntakePower(1.0f);
            intake.setTransferPower(0.6f);
        }
        if(auxilariesTimer.seconds() > 1.5 && auxilariesTimer.seconds() < 2 && intake.getTargetLength() != 0){
            intake.pitchDown();
            intake.setSlidesTargetPosition(125);
            delivery.setSlidesPower(0.75);
        }
        if(auxilariesTimer.seconds() > 2){
            intake.setTransferPower(0.0f);
            isSamplePosessed = true;
            isIntaking = false;
            intakeIsPrepped = false;
        }
    }


    public void transferSample(){
        if(!isTransferring){
            auxilariesTimer.reset();
        }
        isTransferring = true;
        if(auxilariesTimer.seconds() < 0.7 && delivery.getSlideHeight() != 0){
            intake.setIntakePower(0);
            intake.setTransferPower(0);
            delivery.toTransferHeight();
            delivery.setSlidesPower(0.75);
        }
        if(auxilariesTimer.seconds() > 0.7 && auxilariesTimer.seconds() < 0.9 && !delivery.clawClosed){
            delivery.clawClose();
        }
        if(auxilariesTimer.seconds() > 0.9 && auxilariesTimer.seconds() < 2 && delivery.getSlideHeight() != delivery.sampleHeight){
            delivery.toSampleHeight();
            delivery.setSlidesPower(0.75);
        }
        if(auxilariesTimer.seconds() > 2){
            delivery.setPitch(delivery.pitchUpPosition);
            isTransferring = false;
            transferIsPrepped = false;
        }
    }

//    private void relocalizeWithDistanceSensor(){
//        if(!distanceSensorDisabled && distanceSensorX < 0.5){
//            follower.setPose(new Pose(distanceSensorX, follower.getPose().getY(), follower.getPose().getHeading()));
//        }
//    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

//        if(!distanceSensorDisabled) {
//            distanceSensorX = rearDistanceSens.getDistance(DistanceUnit.INCH);
//        }
//        if(distanceSensorX > 60){
//            distanceSensorDisabled = true;
//            rearDistanceSens = null;
//        }
//        relocalizeWithDistanceSensor();

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
//        telemetry.addData("Rear Distance (inches)", distanceSensorX);
        telemetry.addLine();
        telemetry.addData("Is Busy?", follower.isBusy());
        telemetry.addData("Is Delivering?", isDelivering);
        telemetry.addData("Is Intaking?", isIntaking);
        telemetry.addLine();
        telemetry.addData("Is Prepped for Delivering?", deliveryIsPrepped);
        telemetry.addData("Is Prepped for Intaking?", intakeIsPrepped);
        telemetry.addLine();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        delivery = new TwoFishDelivery(this, deliveryTimer);
        intake = new TwoFishIntake(this);


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class); //Applies tuned values to follower and localizer VERY IMPORTANT
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startPoint.getX(), startPoint.getY(), Math.toRadians(-90)));
        buildPaths();

        intake.pitchDown();
        delivery.setWrist(delivery.wristDownPosition);
        delivery.clawClose();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        sleepTimer.reset();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

