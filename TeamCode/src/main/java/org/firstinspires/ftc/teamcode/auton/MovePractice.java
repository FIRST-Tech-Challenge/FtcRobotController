package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "A Move Practice" , group = "Autonomous Pathing Tuning")
public class MovePractice extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path move1;
    private Path move2;
    private Path move3;
    private Path move4;
    private Path move5;
    private Path move6;
    private Path move7;
    private Path move8;
    // TODO: adjust this for each auto
    private Pose startPose = new Pose(7.25, 54.75, 0);
    private Pose spinPose = new Pose(-31, 9, 3.14);

    private int pathState;

    private ElapsedTime pausetime = new ElapsedTime();


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        move1 = new Path(new BezierLine(new Point(7.25,54.75, Point.CARTESIAN), new Point(36.25,45.75, Point.CARTESIAN)));
        move1.setConstantHeadingInterpolation(0); // to spot to hang specimin
        //move2 = new Path(new BezierLine(new Point(36.25,45.75, Point.CARTESIAN), new Point(22.25,87.75, Point.CARTESIAN)));
        move2 = new Path(new BezierCurve(new Point(36.32, 45.75, Point.CARTESIAN), new Point(22.25,87.75, Point.CARTESIAN), new Point(10,80, Point.CARTESIAN)));
        move2.setConstantHeadingInterpolation(-2.35); // to 1st neutral specimin
        move3 = new Path(new BezierLine(new Point(22.25,87.75, Point.CARTESIAN), new Point(17.25,89.75, Point.CARTESIAN)));
        move3.setConstantHeadingInterpolation(-.78); // to basket
        move4 = new Path(new BezierLine(new Point(17.25,89.75, Point.CARTESIAN), new Point(22.25,95.75, Point.CARTESIAN)));
        move4.setConstantHeadingInterpolation(-2.26);  // to 2nd neutral specimin
        move5 = new Path(new BezierLine(new Point(22.25,95.75, Point.CARTESIAN), new Point(17.25,89.75, Point.CARTESIAN)));
        move5.setConstantHeadingInterpolation(-.78);// to basket
        move6 = new Path(new BezierLine(new Point(17.25,89.75, Point.CARTESIAN), new Point(22.25,98.75, Point.CARTESIAN)));
        move6.setConstantHeadingInterpolation(-2.26); // to 3rd neutral specimin
        move7 = new Path(new BezierLine(new Point(22.25,98.75, Point.CARTESIAN), new Point(17.25,89.75, Point.CARTESIAN)));
        move7.setConstantHeadingInterpolation(-.78);// to basket
        move8 = new Path(new BezierLine(new Point(19.25,89.75, Point.CARTESIAN), new Point(62.25,66.75, Point.CARTESIAN)));
        move8.setConstantHeadingInterpolation(-.74);  // to park
       // backwards.setReversed(true);
        follower.followPath(move1);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This is our first trial moves for Into The Deep competition.");
        telemetryA.update();
        pathState = 1;
        follower.setStartingPose(startPose);
        follower.followPath(move1);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        switch (pathState) {
            case 1: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pausetime.reset();
                    pathState = 10;
                }
                break;
            case 10:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(move2);
                    pathState = 11;
                }
                break;
            case 11: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 20;
                }
                break;
            case 20:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(move3);
                    pathState = 21;
                }
                break;
            case 21: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 30;
                }
                break;
            case 30:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(move4);
                    pathState = 31;
                }
                break;
            case 31: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 40;
                }
                break;
            case 40:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(move5);
                    pathState = 41;
                }
                break;
            case 41: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 50;
                }
                break;
            case 50:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(move6);
                    pathState = 51;
                }
                break;
            case 51: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 60;
                }
                break;
            case 60:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(move7);
                    pathState = 61;
                }
                break;
            case 61: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 70;
                }
                break;
            case 70:

                if (pausetime.seconds() > 0.5 ) {
                    follower.setMaxPower(1.0);
                    follower.followPath(move8);
                    pathState = 71;
                }
                break;
            case 71: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 100;
                }
                break;
            case 100:
                break;
        }
        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
