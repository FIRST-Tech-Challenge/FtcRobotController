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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TwoFishDelivery;
import org.firstinspires.ftc.teamcode.TwoFishIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Obs Pedro WIP", group = "Autonomous")
public class ObsPedro extends OpMode {

    ElapsedTime deliveryTimer = new ElapsedTime();

    TwoFishDelivery delivery = null;
    TwoFishIntake intake = null;
    DistanceSensor rearDistanceSens = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int prevPathState;
    private boolean isDelivering = false;
    private boolean isIntaking = false;
    boolean intakeIsPrepped = false;
    boolean deliveryIsPrepped = false;

    boolean distanceSensorDisabled = false;
    double distanceSensorX = 0.0;

    private boolean successfulDeliver = false;
    private boolean successfulIntake = false;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    //Scores the first specimen
//    private final Point startPoint = new Point(8, 56);
//    private final Point toFirstSpecPoint1 = new Point(18, 56);
//    private final Point toFirstSpecPoint2 = new Point(22, 70);
//    private final Point firstSpecScorePoint = new Point(35, 70);

    private final Point startPoint = new Point(8, 61);
    private final Point toFirstSpecPoint1 = new Point(30, 73);
    private final Point toFirstSpecPoint2 = new Point(22, 73);
    private final Point firstSpecScorePoint = new Point(38, 75);

    //Collects the first sample
    private final Point startCollectSample1Point = firstSpecScorePoint;
    private final Point toCollectSample1Point1 = new Point(14, 10);
    private final Point toCollectSample1Point2 = new Point(60, 55);
    private final Point toCollectSample1Point3 = new Point(60, 32);
    private final Point endCollectSample1Point = new Point(20, 24);

    //Collects the second sample
    private final Point startCollectSample2Point = endCollectSample1Point;
    private final Point toCollectSample2Point1 = new Point(44, 32);
    private final Point toCollectSample2Point2 = new Point(50, 32);
    private final Point toCollectSample2Point3 = new Point(55, 20);
    private final Point toCollectSample2Point4 = new Point(30, 20);
    private final Point toCollectSample2Point5 = new Point(25, 28);
    private final Point endCollectSample2Point = new Point(10.5, 33);


    //Scores latter specimens
    private final Point collectSpecPoint = endCollectSample2Point;
    private final Point toSpecPoint1 = new Point(50, 40);
    private final Point toSpecPoint2 = new Point(12, 75);
    private final Point specScorePoint = new Point(39, 73);

    private ElapsedTime auxilariesTimer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();
    private ElapsedTime timeoutTimer = new ElapsedTime();


    private PathChain firstSpecScorePath, firstSampleCollectPath, secondSampleCollectPath, specScorePath, specReturnPath;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        firstSpecScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(startPoint, toFirstSpecPoint1, toFirstSpecPoint2, firstSpecScorePoint))
                .setConstantHeadingInterpolation(0)
                .build();

        firstSampleCollectPath = follower.pathBuilder()
                .addPath(new BezierCurve(startCollectSample1Point, toCollectSample1Point1, toCollectSample1Point2, toCollectSample1Point3))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(toCollectSample1Point3, endCollectSample1Point))
                .setConstantHeadingInterpolation(0)
                .build();

        secondSampleCollectPath = follower.pathBuilder()
                .addPath(new BezierCurve(startCollectSample2Point, toCollectSample2Point1, toCollectSample2Point2, toCollectSample2Point3))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(toCollectSample2Point3, toCollectSample2Point4))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(toCollectSample2Point4, toCollectSample2Point5, endCollectSample2Point))
                .setConstantHeadingInterpolation(0)
                .build();

//        specScorePath = follower.pathBuilder()
//                .addPath(new BezierCurve(collectSpecPoint, toSpecPoint1, toSpecPoint2, specScorePoint))
//                .setConstantHeadingInterpolation(0)
//                .build();
//
//        specReturnPath = follower.pathBuilder()
//                .addPath(new BezierCurve(specScorePoint, toSpecPoint2, toSpecPoint1, collectSpecPoint))
//                .setConstantHeadingInterpolation(0)
//                .build();

        specScorePath = follower.pathBuilder()
                .addPath(new BezierLine(collectSpecPoint, new Point(collectSpecPoint.getX() + 9, collectSpecPoint.getY())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(collectSpecPoint.getX() + 9, collectSpecPoint.getY()), new Point(specScorePoint.getX() - 9, specScorePoint.getY())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(specScorePoint.getX() - 9, specScorePoint.getY()), specScorePoint))
                .setConstantHeadingInterpolation(0)
                .build();

        specReturnPath = follower.pathBuilder()
                .addPath(new BezierLine(specScorePoint, new Point(specScorePoint.getX() - 9, specScorePoint.getY())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(specScorePoint.getX() - 9, specScorePoint.getY()), new Point(collectSpecPoint.getX() + 9, collectSpecPoint.getY())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(collectSpecPoint.getX() + 9, collectSpecPoint.getY()), collectSpecPoint))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                prepDeliverSpecimen();
                follower.followPath(firstSpecScorePath, true);
                if(sleepTimer.seconds() > 1) {
                    setPathState(1);
                    sleepTimer.reset();
                    timeoutTimer.reset();
                }
                break;
            case 1:
                intake.pitchDown();
                if(follower.getPose().getX() > 35) {
                    follower.setMaxPower(0.5);
                } else{
                    follower.setMaxPower(1);
                }
//                if(sleepTimer.seconds() > 0.5)
                if (!follower.isBusy() || timeoutTimer.seconds() > 5) {
//                    follower.setMaxPower(1);
                    //Score first specimen
                    deliverSpecimen();
                    if(!isDelivering) {
                        follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY() + 1));
                        follower.followPath(firstSampleCollectPath, true);

                        if(sleepTimer.seconds() > 1) {
                            setPathState(2);
                            sleepTimer.reset();
                        }
                    }
                }

                break;
            case 2:
                intake.pitchDown();
                if(follower.getPose().getX() < 70 && follower.getPose().getX() > 50) {
                    follower.setMaxPower(0.8);
                } else{
                    follower.setMaxPower(1);
                }
//                if(sleepTimer.seconds() > 0.5)
                if(!follower.isBusy()) {
                    follower.followPath(secondSampleCollectPath);
                    prepIntakeSpecimen();
                    setPathState(3);
                    timeoutTimer.reset();
                }
                break;
            case 3:
                intake.pitchDown();
                if((follower.getPose().getX() < 70 && follower.getPose().getX() > 50) || follower.getPose().getX() < 15) {
                    follower.setMaxPower(0.8);
                } else{
                    follower.setMaxPower(1);
                }

                if(sleepTimer.seconds() > 0.5)
                if(!follower.isBusy() || timeoutTimer.seconds() > 3) {
                    //Collect specimen
                    intakeSpecimen();
                    if(!isIntaking) {
                        follower.followPath(specScorePath, true);
                        if(sleepTimer.seconds() > 2) {
                            prepDeliverSpecimen();
                            setPathState(4);
                            sleepTimer.reset();
                            follower.setMaxPower(1);
                        }
                    }
                }
                break;
            case 4:
                intake.pitchDown();

                if(follower.getPose().getX() > 50) {
                    follower.setMaxPower(0.5);
                } else{
                    follower.setMaxPower(1);
                }

//                if(sleepTimer.seconds() > 0.5)
//                if(follower.getPose().getX() > 59)
//                    deliverSpec();
                if(!follower.isBusy()) {
                    //Score Spec
                    deliverSpecimen();
                    if(!isDelivering) {
                        follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY() + 1));
                        follower.followPath(specReturnPath);
                        if(sleepTimer.seconds() > 1) {
                            prepIntakeSpecimen();
                            setPathState(5);
                            sleepTimer.reset();
                            timeoutTimer.reset();
                        }
                    }
                }
                break;
            case 5:
                intake.pitchDown();
                if(follower.getPose().getX() < 15) {
                    follower.setMaxPower(0.5);
                } else{
                    follower.setMaxPower(1);
                }

                if(sleepTimer.seconds() > 0.5)
                if(!follower.isBusy() || timeoutTimer.seconds() > 3) {
                    //Intake Spec
                    intakeSpecimen();
                    if(!isIntaking) {
                        follower.followPath(specScorePath, true);
                        if(sleepTimer.seconds() > 1) {
                            prepDeliverSpecimen();
                            setPathState(6);
                            sleepTimer.reset();
                        }
                    }
                }
                break;
            case 6:
                intake.pitchDown();

                if(follower.getPose().getX() > 50) {
                    follower.setMaxPower(0.5);
                } else{
                    follower.setMaxPower(1);
                }

                if(sleepTimer.seconds() > 0.5)
                if(!follower.isBusy()) {
                    //Score Spec
                    deliverSpecimen();
                    if(!isDelivering) {
                        follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY() + 1));
                        follower.followPath(specReturnPath);
                        if(sleepTimer.seconds() > 2) {
                            prepIntakeSpecimen();
                            setPathState(7);
                            sleepTimer.reset();
                            timeoutTimer.reset();
                        }
                    }
                }
                break;
            case 7:
                intake.pitchDown();
                if(follower.getPose().getX() < 15) {
                    follower.setMaxPower(0.5);
                } else{
                    follower.setMaxPower(1);
                }

                if(sleepTimer.seconds() > 0.5)
                if(!follower.isBusy() || timeoutTimer.seconds() > 3) {
                    //Intake Spec
                    intakeSpecimen();
                    if(!isIntaking) {
                        follower.followPath(specScorePath, true);
                        prepDeliverSpecimen();
                        setPathState(8);
                    }
                }
                break;
            case 8:
                intake.pitchDown();
                if(follower.getPose().getX() > 50) {
                    follower.setMaxPower(0.5);
                } else{
                    follower.setMaxPower(1);
                }
                    if(!follower.isBusy()) {
                        deliverSpecimen();
                    }
                break;

                //Code loops until match ends
                //Need to add time break
        }
    }


    private void prepDeliverSpecimen(){
        if(!deliveryIsPrepped) {
            deliveryIsPrepped = true;
            delivery.toSpecHeight();
            delivery.setSlidesPower(0.5);
            delivery.setWrist(delivery.wristDownPosition);
            delivery.setPitch(delivery.pitchUpPosition);
        }
    }


    public void deliverSpecimen(){
        isDelivering = true;
        if(delivery.getPitch() != delivery.pitchScoreSpecPosition - 0.00){
            auxilariesTimer.reset();
        }
        if(auxilariesTimer.seconds() < 0.5 && delivery.getPitch() != delivery.pitchScoreSpecPosition){
            delivery.setPitch(delivery.pitchScoreSpecPosition);
            delivery.toSpecHeight();
            delivery.setSlidesPower(1);
        }
        if(auxilariesTimer.seconds() > 0.5 && auxilariesTimer.seconds() < 0.6 && delivery.clawClosed){
            delivery.setClawPosition(delivery.clawOpenPosition + 0.05);
        }
        if(auxilariesTimer.seconds() > 0.6){
            delivery.setSlidesPower(0.5);
            isDelivering = false;
            deliveryIsPrepped = false;
        }
    }



    public void prepIntakeSpecimen() {
        if(!intakeIsPrepped) {
            intakeIsPrepped = true;
            delivery.toMinHeight();
            delivery.setSlidesPower(0.5);
            delivery.setPitch(delivery.pitchIntakePosition);
            delivery.setWrist(delivery.wristUpPosition);
            delivery.clawClose();
            delivery.setClawPosition(delivery.clawOpenPosition + 0.05);
        }
    }

    public void intakeSpecimen(){
        isIntaking = true;
        if(!delivery.clawClosed){
            auxilariesTimer.reset();
        }
        if(auxilariesTimer.seconds() < 0.5 && !delivery.clawClosed){
            delivery.clawClose();
        }
        if(auxilariesTimer.seconds() > 0.5 && auxilariesTimer.seconds() < 0.6 && delivery.getPitch() != delivery.pitchUpPosition){
            delivery.setPitch(delivery.pitchUpPosition);
            delivery.toSpecHeight();
            delivery.setSlidesPower(0.5);
        }
        if(auxilariesTimer.seconds() > 0.6){
            isIntaking = false;
            intakeIsPrepped = false;
        }
    }

    private void relocalizeWithDistanceSensor(){
        if(!distanceSensorDisabled && distanceSensorX < 0.5){
            follower.setPose(new Pose(distanceSensorX, follower.getPose().getY(), follower.getPose().getHeading()));
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
    public void loop() {

        /*
        if(!distanceSensorDisabled) {
            distanceSensorX = rearDistanceSens.getDistance(DistanceUnit.INCH);
        }
        if(distanceSensorX > 60){
            distanceSensorDisabled = true;
            //rearDistanceSens = null;
        }
        relocalizeWithDistanceSensor();
         */

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        delivery.limitCheck();

        // Feedback to Driver Hub
        telemetry.addData("Rear Distance (inches)", distanceSensorX);
        telemetry.addLine();
        telemetry.addData("Target Delivery height", delivery.slideTarget);
        telemetry.addData("DeliverySlide0 height:", delivery.getSlideHeight());
        telemetry.addLine();
        telemetry.addData("Is Busy?", follower.isBusy());
        telemetry.addData("Is Delivering?", isDelivering);
        telemetry.addData("Is Intaking?", isIntaking);
        telemetry.addData("Check Claw Closed",delivery.CheckClawClosed());
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

        rearDistanceSens = hardwareMap.get(DistanceSensor.class, "rearDistanceSens");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class); //Applies tuned values to follower and localizer VERY IMPORTANT
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startPoint.getX(), startPoint.getY(), 0));
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

        intake.pitchDown();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

