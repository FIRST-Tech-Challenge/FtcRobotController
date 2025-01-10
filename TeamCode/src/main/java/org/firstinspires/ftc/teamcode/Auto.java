package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.Pedro.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Pedro.Constants.LConstants;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.Arm;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auto V0.1.0")
public class Auto  extends OpMode {

    private double robotWidth = 12.0472, robotLength = 13.2677;

    private Hardware hardware = new Hardware();
    private Arm arm;
    private GamepadEx controller;
    private Logger logger;

    private final Pose startPose = new Pose((robotLength/2) , (robotWidth/2)+48, Math.toRadians(180));  // Starting position
    private final Pose parkPose = new Pose((robotLength/2), (robotWidth/2)+24, Math.toRadians(180));    // Parking position

    private PathChain park;
    private int pathState = 0;

    private Timer pathTimer;
    private Follower follower;


    @Override
    public void init() {
//        hardware.init(hardwareMap);
//        controller = new GamepadEx(gamepad1);
//        logger = new Logger(telemetry, controller);
//        arm = new Arm(hardware, logger);

        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        follower.followPath(park);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }

    public void buildPaths() {
        park = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                                new Point(73.400, 27.058, Point.CARTESIAN),
                                new Point(81.797, 20.216, Point.CARTESIAN),
                                new Point(12.130, 23.015, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(12.130, 23.015, Point.CARTESIAN),
                                new Point(77.132, 27.369, Point.CARTESIAN),
                                new Point(80.242, 9.019, Point.CARTESIAN),
                                new Point(13.063, 12.130, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(13.063, 12.130, Point.CARTESIAN),
                                new Point(80.242, 13.996, Point.CARTESIAN),
                                new Point(73.400, 5.287, Point.CARTESIAN),
                                new Point(11.819, 2+(robotWidth/2), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }



}
