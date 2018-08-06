package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.QuinticSplineSegment;
import com.acmerobotics.roadrunner.trajectory.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.PathTrajectorySegment;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYSeries;
import org.knowm.xchart.style.MatlabTheme;
import org.knowm.xchart.style.markers.None;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Autonomous
public class SplineFollowTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MyMecanumDrive drive = new MyMecanumDrive(hardwareMap);
        Path path = new Path(new QuinticSplineSegment(
                new QuinticSplineSegment.Waypoint(0.0, 0.0, 60.0, 0.0),
                new QuinticSplineSegment.Waypoint(40.0, 40.0, 60.0, 0.0)
        ));
        DriveConstraints baseConstraints = new DriveConstraints(20.0, 30.0, Math.PI / 2, Math.PI / 2);
        MecanumConstraints constraints = new MecanumConstraints(baseConstraints, drive.getTrackWidth(), drive.getWheelBase());
        Trajectory trajectory = new Trajectory(Collections.singletonList(new PathTrajectorySegment(path, constraints, 250)));
        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(
                drive,
                new PIDCoefficients(0, 0, 0),
                new PIDCoefficients(0, 0, 0),
                0.03631,
                0,
                0);
        File logRoot = LoggingUtil.getLogRoot(this);
        String prefix = "SplineFollowTest-" + System.currentTimeMillis();
        CSVWriter writer = new CSVWriter(new File(logRoot, prefix + ".csv"));

        waitForStart();

        List<Vector2d> targetPositions = new ArrayList<>();
        List<Vector2d> actualPositions = new ArrayList<>();

        double startTime = System.nanoTime() / 1e9;
        follower.followTrajectory(trajectory);
        while (opModeIsActive() && follower.isFollowing()) {
            double time = System.nanoTime() / 1e9;
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d targetPose = trajectory.get(time - startTime);

            writer.put("time", time - startTime);
            writer.put("targetX", targetPose.getX());
            writer.put("targetY", targetPose.getY());
            writer.put("targetHeading", targetPose.getHeading());
            writer.put("currentX", currentPose.getX());
            writer.put("currentY", currentPose.getY());
            writer.put("currentHeading", currentPose.getHeading());
            writer.write();

            targetPositions.add(targetPose.pos());
            actualPositions.add(currentPose.pos());

            follower.update(currentPose);
            drive.updatePoseEstimate();
        }

        double[] targetX = new double[targetPositions.size()];
        double[] targetY = new double[targetPositions.size()];
        double[] actualX = new double[actualPositions.size()];
        double[] actualY = new double[actualPositions.size()];

        for (int i = 0; i < targetX.length; i++) {
            targetX[i] = targetPositions.get(i).getX();
            targetY[i] = targetPositions.get(i).getY();
            actualX[i] = actualPositions.get(i).getX();
            actualY[i] = actualPositions.get(i).getY();
        }

        XYChart chart = new XYChart(600, 400);
        chart.addSeries("Target", targetX, targetY);
        chart.addSeries("Actual", actualX, actualY);
        for (XYSeries series : chart.getSeriesMap().values()) {
            series.setMarker(new None());
        }
        chart.getStyler().setTheme(new MatlabTheme());
        try {
            BitmapEncoder.saveBitmapWithDPI(
                    chart,
                    new File(logRoot, prefix + ".png").getAbsolutePath(),
                    BitmapEncoder.BitmapFormat.PNG,
                    300);
        } catch (IOException e) {
            Log.w("SplineFollowTest", e);
        }

        writer.close();
    }
}
