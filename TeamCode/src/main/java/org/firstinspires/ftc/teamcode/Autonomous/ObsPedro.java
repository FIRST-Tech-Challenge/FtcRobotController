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
    private Follower follower;
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

    //Scores the first specimen
    private final Point startPoint = new Point(8, 56);
    private final Point toFirstSpecPoint1 = new Point(20, 56);
    private final Point toFirstSpecPoint2 = new Point(24, 64);
    private final Point firstSpecScorePoint = new Point(39, 64);

    //Collects the first sample
    private final Point startCollectSample1Point = firstSpecScorePoint;
    private final Point toCollectSample1Point1 = new Point(14, 19);
    private final Point toCollectSample1Point2 = new Point(63, 48);
    private final Point toCollectSample1Point3 = new Point(60, 25);
    private final Point endCollectSample1Point = new Point(22, 24);

    //Collects the first sample
    private final Point startCollectSample2Point = endCollectSample1Point;
    private final Point toCollectSample2Point1 = new Point(44, 32);
    private final Point toCollectSample2Point2 = new Point(60, 29);
    private final Point toCollectSample2Point3 = new Point(57, 14);
    private final Point toCollectSample2Point4 = new Point(22, 14);
    private final Point toCollectSample2Point5 = new Point(22, 28);
    private final Point endCollectSample2Point = new Point(8, 28);


    //Scores latter specimens
    private final Point collectSpecPoint = endCollectSample2Point;
    private final Point toSpecPoint1 = new Point(18, 28);
    private final Point toSpecPoint2 = new Point(24, 66);
    private final Point specScorePoint = new Point(39, 66);


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
                .addPath(new BezierLine(toCollectSample1Point3, endCollectSample1Point))
                .setConstantHeadingInterpolation(0)
                .build();

        secondSampleCollectPath = follower.pathBuilder()
                .addPath(new BezierCurve(startCollectSample2Point, toCollectSample2Point1, toCollectSample2Point2, toCollectSample2Point3))
                .addPath(new BezierLine(toCollectSample2Point3, toCollectSample2Point4))
                .addPath(new BezierCurve(toCollectSample2Point4, toCollectSample2Point5, endCollectSample2Point))
                .setConstantHeadingInterpolation(0)
                .build();

        specScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(collectSpecPoint, toSpecPoint1, toSpecPoint2, specScorePoint))
                .setConstantHeadingInterpolation(0)
                .build();

        specReturnPath = follower.pathBuilder()
                .addPath(new BezierCurve(specScorePoint, toSpecPoint2, toSpecPoint1, collectSpecPoint))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(firstSpecScorePath);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    //Score first specimen
                    follower.followPath(firstSampleCollectPath,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(secondSampleCollectPath,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    //Collect specimen
                    follower.followPath(specScorePath,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    //Score specimen
                    follower.followPath(specScorePath,true);
                    setPathState(3);
                }
                break;
                //Code loops until match ends
                //Need to add time break
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

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startPoint.getX(), startPoint.getY(), 0));
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
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

