package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.KalmanFilterParameters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonPaths1 {

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public static Path movepl;
    public static Path movepr;
    public static Path move1;
    public static Path move2;
    public static Path move3;
    public static Path move4;
    public static Path move5;
    public static Path move6;
    public static Path move7;
    public static Path move8;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutonPaths1 (OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the moves.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init()    {
        // to spot to hang specimen
        movepl = new Path(new BezierLine(
                new Point(9.75,100, Point.CARTESIAN),
                new Point(50,100, Point.CARTESIAN)));
        movepl.setConstantHeadingInterpolation(0); // to spot to hang specimin

        movepr = new Path(new BezierLine(
                new Point(9.75,60, Point.CARTESIAN),
                new Point(49.75,60, Point.CARTESIAN)));
        movepr.setConstantHeadingInterpolation(90); // to spot to hang specimin

        move1 = new Path(new BezierLine(
                new Point(7.25,89.75, Point.CARTESIAN),
                new Point(36.25,80.25, Point.CARTESIAN)));
        move1.setConstantHeadingInterpolation(0); // to spot to hang specimin

        //move2 = new Path(new BezierLine(
        //      new Point(36.25,45.75, Point.CARTESIAN),
        //      new Point(22.25,87.75, Point.CARTESIAN)));
        // move2.setConstantHeadingInterpolation(-2.35); // to 1st neutral specimen

        // to 1st neutral specimen
        move2 = new Path(new BezierCurve(
                new Point(36.32, 80.25, Point.CARTESIAN),
                new Point(8,90, Point.CARTESIAN),
                new Point(36.25,115, Point.CARTESIAN)));
        move2.setTangentHeadingInterpolation();
        move2.setReversed(true);

        // to basket
        move3 = new Path(new BezierLine(
                new Point(22.25,122.25, Point.CARTESIAN),
                new Point(17.25,124.25, Point.CARTESIAN)));
        move3.setConstantHeadingInterpolation(-.78); // to basket

        // to 2nd neutral specimen
        move4 = new Path(new BezierLine(
                new Point(17.25,124.25, Point.CARTESIAN),
                new Point(22.25,130.25, Point.CARTESIAN)));
        move4.setConstantHeadingInterpolation(-2.26);  // to 2nd neutral specimen

        // to basket
        move5 = new Path(new BezierLine(
                new Point(22.25,130.35, Point.CARTESIAN),
                new Point(17.25,124.25, Point.CARTESIAN)));
        move5.setConstantHeadingInterpolation(-.78);// to basket

        // to 3rd neutral specimen
        move6 = new Path(new BezierLine(
                new Point(17.25,124.25, Point.CARTESIAN),
                new Point(22.25,133.25, Point.CARTESIAN)));
        move6.setConstantHeadingInterpolation(-2.26); // to 3rd neutral specimen

        // to basket
        move7 = new Path(new BezierLine(
                new Point(22.25,133.25, Point.CARTESIAN),
                new Point(17.25,124.25, Point.CARTESIAN)));
        move7.setConstantHeadingInterpolation(-.78);// to basket

        // to park
        move8 = new Path(new BezierLine(
                new Point(17.25,124.25, Point.CARTESIAN),
                new Point(62.25,101.25, Point.CARTESIAN)));
        move8.setConstantHeadingInterpolation(-.74);  // to park
    }
}
