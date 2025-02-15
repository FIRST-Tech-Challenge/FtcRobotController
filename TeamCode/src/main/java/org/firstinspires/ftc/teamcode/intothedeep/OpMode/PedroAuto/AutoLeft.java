package org.firstinspires.ftc.teamcode.intothedeep.OpMode.PedroAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.common.Helper;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.ClawRotor;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.intothedeep.Subsystems.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Auto Left Sample", group = "A Into the Deep")

public class AutoLeft extends LinearOpMode {

    //our robot subsystems
    Slide slide;
    Claw claw;
    ClawRotor clawRotor;
    OuttakeArm outtakeArm;
    Intake intake;
    IntakeSlide intakeSlide;
    private DigitalChannel touchSensorFrontLimitRight;

    //Pedro pathing
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    //the state of the auto
    private int pathState;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.25, 109.75, Math.toRadians(0));

    private final Pose scorePose = new Pose(11, 124.5, Math.toRadians(-45));//11,124
    private final Pose scorePose2 = new Pose(15, 127, Math.toRadians(-45));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(23, 119, Math.toRadians(0));//23

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(23, 128, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(26.5, 123, Math.toRadians(45));//25

    /** Alliance's preloaded sample */
    private final Pose pickup4Pose = new Pose(18, 90, Math.toRadians(-135));

    /** Starting position to pickup a sample in the submersible*/
    private final Pose submersibleCurve = new Pose(80, 115, Math.toRadians(-90));
    private final Pose submersiblePickupPose1 = new Pose(62, 106, Math.toRadians(-90));
    private final Pose submersiblePickupPose2 = new Pose(62, 98, Math.toRadians(-90));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 92, Math.toRadians(-90));//60, 92

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(75, 110, Math.toRadians(-90));

    private Path scorePreload, park;
    private Path submersibleCurvePath;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    private PathChain grabPickup4, scorePickup4, submersiblePath1, submersiblePath2, submersiblePath3, submersibleScore;


    //the speed to extend and retrack intake
    private final double intakeOutSpeed = 0.65;
    //It can be faster if aiming for 1+4 or 1+5
    private final double intakeInSpeed = 0.3;//0.4

    //slide up wait time, slide needs up before we
    //can rotate the arm
    private final double slideUpWaitTime = 300; //300
    private final double slideFullyUpWaitTime = 800;//900, 1000

    private final double clawOpenWaitTime = 250;
    //the time takes to extend intake
    private final double extendingIntakeWaitTime = 1100;//1200, 900
    //the time to wait while intake is extended
    private final double retrackingIntakeWaitTime = 1100;
    private boolean intakeSlided = false;

    Log log;
    @Override
    public void runOpMode() throws InterruptedException {

        log = new Log("IntoTheDeep", true);

        slide = new Slide(hardwareMap, this);
        slide.runWithEncoder();

        claw = new Claw(hardwareMap, this);
        claw.close();

        clawRotor = new ClawRotor(hardwareMap, this);
        //clawRotor.SetClawDown();

        outtakeArm = new OuttakeArm(hardwareMap, this);
        outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
        intake = new Intake(hardwareMap, this);
        //intake.MoveToOuttakePosition();

        intakeSlide = new IntakeSlide(hardwareMap);
        intakeSlide.Move(0.49);

        touchSensorFrontLimitRight =  hardwareMap.get(DigitalChannel.class, "frontLimitRight");
        touchSensorFrontLimitRight.setMode(DigitalChannel.Mode.INPUT);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        setPathState(0);

        buildPaths();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        /** Waiting for Play button being touched */
        waitForStart();

        if (isStopRequested()) return;

        /** Play button is touched, Auto starts */


        while (!isStopRequested()  && opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            slide.autoMoveToWithoutWaitingLoop();

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        log.close();
    }

    private void autonomousPathUpdate()
    {
        //double headingDelta = 0;
        double poseDeltaX = 0;
        double poseDeltaY = 0;

        switch (pathState) {
            /**Pre load*/
            case 0: //slide up first
                slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1);
                actionTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if(actionTimer.getElapsedTime() >= slideUpWaitTime) {
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_DELIVERY_POSITION);
                    follower.followPath(scorePreload);
                    setPathState(2);
                }
                break;
            case 2: //at score position, slide up, arm up
                poseDeltaX = Math.abs(follower.getPose().getX() - scorePose.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - scorePose.getY());

                if (poseDeltaX <= 1 && poseDeltaY <= 1) {
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3: //open claw to score
                if(actionTimer.getElapsedTime() >= slideFullyUpWaitTime)//1200
                {
                    claw.open();
                    //intake.MoveToIntakePosition();
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4: //Claw fully opened, reset arm
                if(actionTimer.getElapsedTime() >= clawOpenWaitTime) {
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
                    actionTimer.resetTimer();

                    setPathState(5);
                }
                break;

                /** #1 */
            case 5://move to pickup 1 position
                if(actionTimer.getElapsedTime() >= 100) {
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1);

                    intake.MoveToIntakePosition();
                    follower.followPath(grabPickup1, true);
                    setPathState(6);
                }
                break;
            case 6: //at score position, slide up, arm up
                poseDeltaX = Math.abs(follower.getPose().getX() - pickup1Pose.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - pickup1Pose.getY());

                if (poseDeltaX <= 1 && poseDeltaY <= 1) {
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    intakeSlide.Move(intakeOutSpeed);

                    actionTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if(actionTimer.getElapsedTime() > extendingIntakeWaitTime)
                {
                    intake.MoveToOuttakePosition();
                    intakeSlide.Move(intakeInSpeed);
                    actionTimer.resetTimer();

                    setPathState(8);
                }
                break;
            case 8:
                if(intakeSlide.isTouchSensorPressed() && !intakeSlided)
                {
                    intakeSlided = true;
                    claw.close();
                    sleep(100);
                    intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1);
                    actionTimer.resetTimer();

                    setPathState(9);
                }
                break;
            case 9:
                if(actionTimer.getElapsedTime() >= slideUpWaitTime) {//500
                    intakeSlided = false;
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_DELIVERY_POSITION);
                    follower.followPath(scorePickup1, true);
                    setPathState(10);
                }
                break;
            case 10:
                poseDeltaX = Math.abs(follower.getPose().getX() - scorePose2.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - scorePose2.getY());

                if (poseDeltaX <= 1 && poseDeltaY <= 1) {
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11: //open claw to score
                if(actionTimer.getElapsedTime() >= slideFullyUpWaitTime)
                {
                    claw.open();
                    //intake.MoveToIntakePosition();
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12: //Claw fully opened, reset arm
                if(actionTimer.getElapsedTime() >= clawOpenWaitTime) {
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
                    actionTimer.resetTimer();

                    setPathState(13);
                }
                break;
            case 13:
                if(actionTimer.getElapsedTime() >= 100) {
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1);

                    intake.MoveToIntakePosition();

                    follower.followPath(grabPickup2, true);
                    setPathState(14);
                }
                break;

                /** #2 */
            case 14: //at score position, slide up, arm up
                poseDeltaX = Math.abs(follower.getPose().getX() - pickup2Pose.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - pickup2Pose.getY());

                if (poseDeltaX <= 1.2 && poseDeltaY <= 1.2) {
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    intakeSlide.Move(intakeOutSpeed);

                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                if(actionTimer.getElapsedTime() > extendingIntakeWaitTime)
                {
                    intake.MoveToOuttakePosition();
                    intakeSlide.Move(intakeInSpeed);
                    actionTimer.resetTimer();

                    setPathState(16);
                }
                break;
            case 16:
                if(intakeSlide.isTouchSensorPressed() && !intakeSlided)
                {
                    intakeSlided = true;
                    claw.close();
                    sleep(100);
                    intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1);
                    actionTimer.resetTimer();

                    setPathState(17);
                }
                break;
            case 17:
                if(actionTimer.getElapsedTime() >= slideUpWaitTime) {
                    intakeSlided = false;
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_DELIVERY_POSITION);
                    follower.followPath(scorePickup2, true);
                    setPathState(18);
                }
                break;
            case 18:
                poseDeltaX = Math.abs(follower.getPose().getX() - scorePose2.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - scorePose2.getY());

                if (poseDeltaX <= 1.2 && poseDeltaY <= 1.2) {
                    actionTimer.resetTimer();
                    setPathState(19);
                }
                break;
            case 19: //open claw to score
                if(actionTimer.getElapsedTime() >= slideFullyUpWaitTime)
                {
                    claw.open();
                    //intake.MoveToIntakePosition();
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;
            case 20: //Claw fully opened, reset arm
                if(actionTimer.getElapsedTime() >= clawOpenWaitTime) {
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
                    actionTimer.resetTimer();

                    setPathState(21);
                }
                break;
            case 21:
                if(actionTimer.getElapsedTime() >= 100) {
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1);

                    intake.MoveToIntakePosition();
                    follower.followPath(grabPickup3, true);
                    setPathState(22);
                }
                break;


                /** #3 */
            case 22: //at score position, slide up, arm up
                poseDeltaX = Math.abs(follower.getPose().getX() - pickup3Pose.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - pickup3Pose.getY());

                if (poseDeltaX <= 1 && poseDeltaY <= 1) {
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    intakeSlide.Move(intakeOutSpeed);

                    actionTimer.resetTimer();
                    setPathState(23);
                }
                break;
            case 23:
                if(actionTimer.getElapsedTime() > extendingIntakeWaitTime)
                {
                    intake.MoveToOuttakePosition();
                    intakeSlide.Move(intakeInSpeed);
                    actionTimer.resetTimer();

                    setPathState(24);
                }
                break;
            case 24:
                if(intakeSlide.isTouchSensorPressed() && !intakeSlided)
                {
                    intakeSlided = true;
                    claw.close();
                    sleep(100);
                    intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1);
                    actionTimer.resetTimer();

                    setPathState(25);
                }
                break;
            case 25:
                if(actionTimer.getElapsedTime() >= slideUpWaitTime) {
                    intakeSlided = false;
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_DELIVERY_POSITION);
                    follower.followPath(scorePickup3, true);
                    setPathState(26);
                }
                break;
            case 26:
                poseDeltaX = Math.abs(follower.getPose().getX() - scorePose2.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - scorePose2.getY());

                if (poseDeltaX <= 1 && poseDeltaY <= 1) {
                    actionTimer.resetTimer();
                    setPathState(27);
                }
                break;
            case 27: //open claw to score
                if(actionTimer.getElapsedTime() >= slideFullyUpWaitTime)//1500
                {
                    claw.open();
                    //intake.MoveToIntakePosition();
                    actionTimer.resetTimer();
                    setPathState(28); //50 will do the parking only from here
                }
                break;

                /**Pick one from submersible*/
            case 28:
                if(actionTimer.getElapsedTime() >= clawOpenWaitTime) {
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
                    actionTimer.resetTimer();
                    setPathState(29);
                }
                break;
            case 29:
                if(actionTimer.getElapsedTime() >= 100) {
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1);
                    intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);
                    intakeSlide.Move(intakeOutSpeed);
                    //intake.MoveToOuttakePosition();
                    follower.followPath(submersibleCurvePath, true);
                    setPathState(30);
                }
                break;
            case 30:
                //poseDeltaX = Math.abs(follower.getPose().getX() - parkPose.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - submersibleCurve.getY());
              //  poseDeltaY = Math.abs(follower.getPose().getY() - submersibleCurvePath.getY());
                if (poseDeltaY <= 1) {
                    follower.followPath(submersiblePath2, true);
                    setPathState(31);
                }
                break;
            case 31:
                //poseDeltaX = Math.abs(follower.getPose().getX() - parkPose.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - submersiblePickupPose2.getY());
                if (poseDeltaY <= 1) {
                    intakeSlide.Move(intakeOutSpeed);
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    actionTimer.resetTimer();
                    setPathState(32);
                }
                break;
            case 32:
                if(actionTimer.getElapsedTime() >= 500){
                    intake.MoveToReadyPosition();
                    actionTimer.resetTimer();
                    setPathState(33);
                }
                break;
            case 33:
                if(actionTimer.getElapsedTime() > 1500){//1500
                    follower.followPath(submersiblePath3, true);
                    setPathState(34);
                }
                break;
            case 34:
                //poseDeltaX = Math.abs(follower.getPose().getX() - parkPose.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - submersiblePickupPose1.getY());
                if (poseDeltaY <= 1) {

                    intake.MoveToOuttakePosition();
                    actionTimer.resetTimer();
                    setPathState(100);
                }
                break;
            case 100:
                if(actionTimer.getElapsedTime()>=1000){
                    intake.MoveToOuttakePosition();
                    setPathState(35);
                }
            case 35:
                if(actionTimer.getElapsedTime() > 100){
                    intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
                    actionTimer.resetTimer();
                    setPathState(36);
                }
                break;
            case 36:
                if(actionTimer.getElapsedTime() > 200){
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    intakeSlide.Move(intakeInSpeed);
                    actionTimer.resetTimer();
                    setPathState(37);
                }
                break;
            case 37:
                if(intakeSlide.isTouchSensorPressed() && !intakeSlided){
                    intakeSlided = true;
                    claw.close();
                    sleep(100);
                    intake.SetIntakeSpinner(Intake.IntakeMode.OUT);
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.HIGH_BASkET, 1);

                    follower.followPath(submersibleScore, true);
                    actionTimer.resetTimer();
                    setPathState(38);
                }
                break;
            case 38:
                if(actionTimer.getElapsedTime() >= slideUpWaitTime) {
                    intakeSlided = false;
                    intake.SetIntakeSpinner(Intake.IntakeMode.IN);
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_DELIVERY_POSITION);
                    setPathState(39);
                }
                break;
            case 39:
                poseDeltaY = Math.abs(follower.getPose().getY() - scorePose2.getY());
                poseDeltaX = Math.abs(follower.getPose().getX() - scorePose2.getX());
                if (poseDeltaY <= 1.2){

                    setPathState(40);
                }
                break;
            case 40: //open claw to score
                if(actionTimer.getElapsedTime() >= slideFullyUpWaitTime)
                {
                    claw.open();
                    //intake.MoveToIntakePosition();
                    actionTimer.resetTimer();
                    setPathState(41);
                }
                break;
            case 41: //Claw fully opened, reset arm
                if(actionTimer.getElapsedTime() >= clawOpenWaitTime) {
                    outtakeArm.RotateTo(outtakeArm.SAMPLE_PICKUP_POSITION);
                    actionTimer.resetTimer();

                    setPathState(42);
                }
                break;
            case 42:
                if(actionTimer.getElapsedTime() >= 100) {
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1);
                    intake.MoveToIntakePosition();
                    actionTimer.resetTimer();
                    setPathState(43);
                }
                break;
            /** pick 2nd from submersible*/
           /* case 43:
                if(actionTimer.getElapsedTime() >retrackingIntakeWaitTime){
                    follower.followPath(submersiblePath1);
                    actionTimer.resetTimer();
                    setPathState(44);
                }
            case 44:
                poseDeltaY = Math.abs(follower.getPose().getY() - submersiblePickupPose1.getY());
                poseDeltaX = Math.abs(follower.getPose().getX() - submersiblePickupPose1.getX());
                if (poseDeltaX <= 1 && poseDeltaY <= 1) {
                    actionTimer.resetTimer();
                    setPathState(45);
                }
            case 45:
           */


            /** Park */
            case 80: //Claw fully opened, reset arm
                if(actionTimer.getElapsedTime() >= clawOpenWaitTime) {
                    outtakeArm.RotateTo(outtakeArm.SPECIMEN_PARK_POSITION1);
                    actionTimer.resetTimer();

                    setPathState(81);
                }
                break;
            case 81:
                if(actionTimer.getElapsedTime() >= 100) {
                    slide.moveToPredefinedPositionWithoutWaiting(Slide.SlideTargetPosition.DOWN, 1);
                    intake.SetIntakeSpinner(Intake.IntakeMode.IDLE);
                    intake.MoveToOuttakePosition();
                    follower.followPath(park, true);
                    setPathState(82);
                }
                break;
            case 82:
                //poseDeltaX = Math.abs(follower.getPose().getX() - parkPose.getX());
                poseDeltaY = Math.abs(follower.getPose().getY() - parkPose.getY());
                if (poseDeltaY <= 1 || !touchSensorFrontLimitRight.getState()) {
                    //actionTimer.resetTimer();
                    outtakeArm.RotateTo(outtakeArm.SPECIMEN_PARK_POSITION2);
                    setPathState(83);
                }
                break;
        }
    }

    private void buildPaths() {
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
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
                .build();

        //# 4 opional
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup4Pose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup4Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup4Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), scorePose2.getHeading())
                .build();


        submersibleCurvePath = new Path(new BezierCurve(new Point(scorePose2), new Point(submersibleCurve), new Point(submersiblePickupPose1)));
        submersibleCurvePath.setLinearHeadingInterpolation(scorePose2.getHeading(), submersiblePickupPose1.getHeading());

        submersiblePath2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(submersiblePickupPose1), new Point(submersiblePickupPose2)))
                .setLinearHeadingInterpolation(submersiblePickupPose1.getHeading(), submersiblePickupPose2.getHeading())
                .build();
        submersiblePath3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(submersiblePickupPose2), new Point(submersiblePickupPose1)))
                .setLinearHeadingInterpolation(submersiblePickupPose2.getHeading(), submersiblePickupPose1.getHeading())
                .build();
        submersibleScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(submersiblePickupPose1), new Point(scorePose2)))
                .setLinearHeadingInterpolation(submersiblePickupPose1.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose2), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose2.getHeading(), parkPose.getHeading());
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    private void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    private double calculateHeadingDelta(Pose currentPose, Pose targetPose)
    {
        double headingDelta = currentPose.getHeading() - targetPose.getHeading();
        headingDelta = Math.toDegrees(Helper.normDelta(headingDelta));

//        String msg = "C: " + Math.toDegrees(follower.getPose().getHeading()) +", T: " +
//                Math.toDegrees(pickup1Pose.getHeading()) + ", Delat: " + headingDelta;
//
//        log.addData(msg);
//        log.update();

        return headingDelta;
    }
}




