package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name="BlueLeftPedro", group="Auto")
public class BlueLeftPedro extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Follower follower = new Follower(hardwareMap);
        Pose startPose = new Pose(8, 108, Math.toRadians(0));  // Starting position
        PathChain path = new BlueLeftPath().getPath();
        follower.setStartingPose(startPose);

        // Define paths separately
        PathChain path1 = new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(8.000, 108.000, Point.CARTESIAN),
                        new Point(18.000, 118.000, Point.CARTESIAN),
                        new Point(15.000, 125.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        PathChain path2 = new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(15.000, 125.000, Point.CARTESIAN),
                        new Point(29.000, 120.000, Point.CARTESIAN),
                        new Point(32.000, 121.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        PathChain path3 = new PathBuilder()
                .addPath(new BezierLine(
                        new Point(32.000, 121.000, Point.CARTESIAN),
                        new Point(15.000, 125.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        PathChain path4 = new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(15.000, 125.000, Point.CARTESIAN),
                        new Point(25.000, 123.000, Point.CARTESIAN),
                        new Point(30.000, 128.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(10))
                .build();

        PathChain path5 = new PathBuilder()
                .addPath(new BezierLine(
                        new Point(30.000, 128.000, Point.CARTESIAN),
                        new Point(15.000, 125.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(-45))
                .build();

        PathChain path6 = new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(15.000, 125.000, Point.CARTESIAN),
                        new Point(23.000, 125.000, Point.CARTESIAN),
                        new Point(27.000, 131.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(27))
                .build();

        PathChain path7 = new PathBuilder()
                .addPath(new BezierLine(
                        new Point(27.000, 131.000, Point.CARTESIAN),
                        new Point(15.000, 125.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(27), Math.toRadians(-45))
                .build();

        // Wait for the start command
        waitForStart();

        // Execute paths sequentially with a half-second delay
//        executePath(follower, path1);
//        executePath(follower, path2);
//        executePath(follower, path3);
//        executePath(follower, path4);
//        executePath(follower, path5);
//        executePath(follower, path6);
//        executePath(follower, path7);

        follower.followPath(path);
    }

    private void executePath(Follower follower, PathChain path) throws InterruptedException {
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // Add telemetry for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        // Pause for half a second
        sleep(500);
    }
}


