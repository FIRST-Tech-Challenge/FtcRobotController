package org.firstinspires.ftc.teamcode.robots.reachRefactor.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.List;

public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in
    private static final double VELOCITY_SCALE = 0.1;

    private static final String ROBOT_COLOR = "Black";
    private static final String WHEEL_COLOR = "Red";

    public static void drawLine(Canvas canvas, Vector2d p1, Vector2d p2) {
        canvas.strokeLine(
                p1.getX(),
                p1.getY(),
                p2.getX(),
                p2.getY()
        );
    }

    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
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
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawPose(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose, double chassisLength, double swivelAngle, List<Double> wheelVelocities) {
        // calculating wheel positions
        Vector2d position = pose.vec();
        Vector2d leftWheel = new Vector2d(0, Constants.TRACK_WIDTH / 2);
        Vector2d rightWheel = new Vector2d(0, -Constants.TRACK_WIDTH / 2);
        Vector2d swerveWheel = new Vector2d(-chassisLength, 0);

        // calculating wheel vectors
        Vector2d leftWheelEnd = leftWheel.plus(new Vector2d(VELOCITY_SCALE * wheelVelocities.get(0), 0));
        Vector2d rightWheelEnd = rightWheel.plus(new Vector2d(VELOCITY_SCALE * wheelVelocities.get(1), 0));
        Vector2d swerveWheelEnd = swerveWheel.plus(new Vector2d(0, -VELOCITY_SCALE * wheelVelocities.get(2)).rotated(swivelAngle));

        // rotating points by heading, translating by position
        double heading = pose.getHeading();
        leftWheel = position.plus(leftWheel.rotated(heading));
        rightWheel = position.plus(rightWheel.rotated(heading));
        swerveWheel = position.plus(swerveWheel.rotated(heading));
        leftWheelEnd = position.plus(leftWheelEnd.rotated(heading));
        rightWheelEnd = position.plus(rightWheelEnd.rotated(heading));
        swerveWheelEnd = position.plus(swerveWheelEnd.rotated(heading));

        // drawing axles
        canvas.setStroke(ROBOT_COLOR);
        drawLine(canvas, leftWheel, rightWheel); // front axle
        drawLine(canvas, position, swerveWheel); // linear slides

        // drawing wheels
        canvas.setStroke(WHEEL_COLOR);
        drawLine(canvas, leftWheel, leftWheelEnd);
        drawLine(canvas, rightWheel, rightWheelEnd);
        drawLine(canvas, swerveWheel, swerveWheelEnd);
    }
}
