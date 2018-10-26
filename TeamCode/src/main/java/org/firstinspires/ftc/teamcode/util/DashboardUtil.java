package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final int DEFAULT_SAMPLES = 1000;

    public static void drawSampledTrajectory(Canvas canvas, Trajectory trajectory, int samples) {
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dt = trajectory.duration() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double time = i * dt;
            Pose2d pose = trajectory.get(time);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledTrajectory(Canvas canvas, Trajectory trajectory) {
        drawSampledTrajectory(canvas, trajectory, DEFAULT_SAMPLES);
    }

    public static void drawSampledPath(Canvas canvas, Path path, int samples) {
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_SAMPLES);
    }
}
