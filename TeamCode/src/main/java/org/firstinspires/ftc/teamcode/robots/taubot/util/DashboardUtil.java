package org.firstinspires.ftc.teamcode.robots.taubot.util;

import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.wrapAngleRad;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import org.firstinspires.ftc.teamcode.robots.taubot.vision.Target;
import org.firstinspires.ftc.teamcode.util.Vector3;

import java.util.List;

@Config
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in
    private static final double TURRET_RADIUS = 6;
    private static final double WRIST_RADIUS = 3;
    public static double VELOCITY_SCALE = 0.4;

    private static final String ROBOT_COLOR = "Black";
    private static final String WHEEL_COLOR = "Red";
    private static final String TURRET_COLOR = "#00ff44"; //bright green

    private static final String TARGET_COLOR = "Orange";


    private static final String EXTEND_COLOR = "Purple";
    private static final String SHOULDER_TO_ELBOW_COLOR = "Purple";
    private static final String ELBOW_TO_WRIST_COLOR = "Orange";

    private static double TRACK_WIDTH = 14;

    public static void drawLine(Canvas canvas, Vector2d p1, Vector2d p2) {
        canvas.strokeLine(
                p1.getX(),
                p1.getY(),
                p2.getX(),
                p2.getY()
        );
    }

    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        canvas.setStroke("#3F51B5");

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

    public static void drawRobot(Canvas canvas, Pose2d pose, List<Double> wheelVelocities, double turretHeading, double shoulderAngle, double extendInches, Vector3 fieldPositionTarget, List<Target> targets) {
        // calculating wheel positions
        Vector2d position = pose.vec();
        Vector2d leftWheel = new Vector2d(0, TRACK_WIDTH / 2);
        Vector2d rightWheel = new Vector2d(0, -TRACK_WIDTH / 2);

        // calculating wheel vectors
        Vector2d leftWheelEnd = leftWheel.plus(new Vector2d(VELOCITY_SCALE * wheelVelocities.get(0), 0));
        Vector2d rightWheelEnd = rightWheel.plus(new Vector2d(VELOCITY_SCALE * wheelVelocities.get(1), 0));

        // rotating points by heading, translating by position
        double heading = pose.getHeading();
        leftWheel = position.plus(leftWheel.rotated(heading));
        rightWheel = position.plus(rightWheel.rotated(heading));

        leftWheelEnd = position.plus(leftWheelEnd.rotated(heading));
        rightWheelEnd = position.plus(rightWheelEnd.rotated(heading));

        //draw current field target
        canvas.setStroke(TARGET_COLOR);
        canvas.strokeCircle(fieldPositionTarget.x, fieldPositionTarget.y, 1);
        canvas.strokeCircle(fieldPositionTarget.x, fieldPositionTarget.y, 3);

        // drawing axles
        canvas.setStroke(ROBOT_COLOR);
        drawLine(canvas, leftWheel, rightWheel); // front axle

        //draw chassis
        canvas.setStroke(ROBOT_COLOR);
        drawPose(canvas,pose);

        // drawing wheels
        canvas.setStroke(WHEEL_COLOR);
        drawLine(canvas, leftWheel, leftWheelEnd);
        drawLine(canvas, rightWheel, rightWheelEnd);

        // drawing turret
        canvas.setStroke(TURRET_COLOR);
        Vector2d turretPose = position.minus(
                new Vector2d(
                        3, //turret offset from center
                        0
                ).rotated(heading)
        );

        canvas.strokeCircle(turretPose.getX(), turretPose.getY(), TURRET_RADIUS);
        turretHeading = Math.toRadians(turretHeading);
        Vector2d u = new Vector2d(Math.cos(turretHeading), Math.sin(turretHeading));
        Vector2d v = u.times(TURRET_RADIUS);
        double x1 = turretPose.getX() + v.getX() / 2, y1 = turretPose.getY() + v.getY() / 2;
        double x2 = turretPose.getX() + v.getX(), y2 = turretPose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);

        // drawing crane
        shoulderAngle = wrapAngleRad(Math.toRadians(90 - shoulderAngle));
        //elbowAngle = wrapAngleRad(Math.toRadians(180 - (elbowAngle + Math.toDegrees(shoulderAngle))));
        //wristAngle = wrapAngleRad(Math.toRadians(180) - wrapAngleRad(elbowAngle + Math.toRadians(wristAngle)));

        canvas.setStroke(EXTEND_COLOR);
        Vector2d extension = u.times(extendInches * Math.cos(shoulderAngle));
        double ste_x1 = turretPose.getX(), ste_y1 = turretPose.getY();
        double ste_x2 = turretPose.getX() + extension.getX(), ste_y2 = turretPose.getY() + extension.getY();
        canvas.strokeLine(ste_x1, ste_y1, ste_x2, ste_y2);

        //draw targets
        canvas.setStroke(TARGET_COLOR);
        for (Target can: targets) {
            canvas.strokeCircle(can.getFieldPosition().getX(), can.getFieldPosition().getY(), 1.8);
        }

/*
        canvas.setStroke(ELBOW_TO_WRIST_COLOR);
        Vector2d elbowToWrist = u.times(ELBOW_TO_WRIST * Math.cos(elbowAngle));
        double etw_x1 = ste_x2, etw_y1 = ste_y2;
        double etw_x2 = etw_x1 + elbowToWrist.getX(), etw_y2 = ste_y2 + elbowToWrist.getY();
        canvas.strokeLine(etw_x1, etw_y1, etw_x2, etw_y2);

        String wristColor = wristAngle > Math.toRadians(0) && wristAngle < Math.toRadians(180) ? "#00FF00" : "#FF0000";
        canvas.setStroke(wristColor);
        canvas.strokeCircle(etw_x2, etw_y2, WRIST_RADIUS);
  */


    }
}
