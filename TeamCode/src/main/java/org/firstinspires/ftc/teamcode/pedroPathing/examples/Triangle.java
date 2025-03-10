package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
@Autonomous(name = "Triangle", group = "Examples")
public class Triangle extends OpMode {
    private Follower follower;

    private final Pose startPose = new Pose(0,0, Math.toRadians(0));
    private final Pose interPose = new Pose(24, -24, Math.toRadians(90));
    private final Pose endPose = new Pose(24, 24, Math.toRadians(45));

    private PathChain triangle;

    private Telemetry telemetryA;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }

        follower.telemetryDebug(telemetryA);
    }

    /**
     * This initializes the Follower and creates the PathChain for the "triangle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(interPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(new Point(interPose), new Point(endPose)))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(new Point(endPose), new Point(startPose)))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(triangle);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly triangular shape,"
                + "starting on the bottom-middle point. So, make sure you have enough "
                + "space to the left, front, and right to run the OpMode.");
        telemetryA.update();
    }

}
