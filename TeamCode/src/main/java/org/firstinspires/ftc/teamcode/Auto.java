package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    private Telemetry telemetryA;

    private final Pose startPose = new Pose((robotLength/2) , (robotWidth/2)+48, Math.toRadians(180));  // Starting position

    private Path park;
    private Follower follower;


    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        follower.followPath(park, true);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Auto Initialized");
        telemetryA.update();
    }

    @Override
    public void loop() {
        follower.update();
        follower.telemetryDebug(telemetryA);
    }

    public void buildPaths() {
        park = new Path(
                new BezierCurve(
                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                new Point(20.0, 53.0, Point.CARTESIAN),
                new Point(32.0, 73.6, Point.CARTESIAN),
                new Point(46 - (robotLength/2), 72.0, Point.CARTESIAN)
                ));

        park.setConstantHeadingInterpolation(Math.toRadians(180));
    }



}
