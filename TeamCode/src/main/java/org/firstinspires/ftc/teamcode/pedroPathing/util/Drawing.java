package org.firstinspires.ftc.teamcode.pedroPathing.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

/**
 * This is the Drawing class. It handles the drawing of stuff on FTC Dashboard, like the robot.
 *
 * @author Logan Nash
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/22/2024
 */
public class Drawing {
    private static TelemetryPacket packet;

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     * @param follower
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), "#3F51B5");
            Point closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), "#3F51B5");
        }
        drawPoseHistory(follower.getDashboardPoseTracker(), "#4CAF50");
        drawRobot(follower.getPose(), "#4CAF50");
        sendPacket();
    }

    /**
     * This adds instructions to the current packet to draw a robot at a specified Pose with a specified
     * color. If no packet exists, then a new one is created.
     *
     * @param pose the Pose to draw the robot at
     * @param color the color to draw the robot with
     */
    public static void drawRobot(Pose pose, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        Drawing.drawRobotOnCanvas(packet.fieldOverlay(), pose.copy());
    }

    /**
     * This adds instructions to the current packet to draw a Path with a specified color. If no
     * packet exists, then a new one is created.
     *
     * @param path the Path to draw
     * @param color the color to draw the Path with
     */
    public static void drawPath(Path path, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        Drawing.drawPath(packet.fieldOverlay(), path.getDashboardDrawingPoints());
    }

    /**
     * This adds instructions to the current packet to draw all the Paths in a PathChain with a
     * specified color. If no packet exists, then a new one is created.
     *
     * @param pathChain the PathChain to draw
     * @param color the color to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, String color) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), color);
        }
    }

    /**
     * This adds instructions to the current packet to draw the pose history of the robot. If no
     * packet exists, then a new one is created.
     *
     * @param poseTracker the DashboardPoseTracker to get the pose history from
     * @param color the color to draw the pose history with
     */
    public static void drawPoseHistory(DashboardPoseTracker poseTracker, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().strokePolyline(poseTracker.getXPositionsArray(), poseTracker.getYPositionsArray());
    }

    /**
     * This tries to send the current packet to FTC Dashboard.
     *
     * @return returns if the operation was successful.
     */
    public static boolean sendPacket() {
        if (packet != null) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            packet = null;
            return true;
        }
        return false;
    }

    /**
     * This draws a robot on the Dashboard at a specified Point.
     *
     * @param c the Canvas on the Dashboard on which this will draw
     * @param t the Point to draw at
     */
    public static void drawRobotOnCanvas(Canvas c, Point t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

        Vector halfv = new Vector(0.5*ROBOT_RADIUS, t.getTheta());
        Vector p1 = MathFunctions.addVectors(halfv, new Vector(t.getR(), t.getTheta()));
        Vector p2 = MathFunctions.addVectors(p1, halfv);
        c.strokeLine(p1.getXComponent(), p1.getYComponent(), p2.getXComponent(), p2.getYComponent());
    }

    /**
     * This draws a robot on the Dashboard at a specified Pose. This is more useful for drawing the
     * actual robot, since the Pose contains the direction the robot is facing as well as its position.
     *
     * @param c the Canvas on the Dashboard on which this will draw
     * @param t the Pose to draw at
     */
    public static void drawRobotOnCanvas(Canvas c, Pose t) {
        final double ROBOT_RADIUS = 9;

        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);
        Vector v = t.getHeadingVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = t.getX() + v.getXComponent() / 2, y1 = t.getY() + v.getYComponent() / 2;
        double x2 = t.getX() + v.getXComponent(), y2 = t.getY() + v.getYComponent();
        c.strokeLine(x1, y1, x2, y2);
    }

    /**
     * This draws a Path on the Dashboard from a specified Array of Points.
     *
     * @param c the Canvas on the Dashboard on which this will draw
     * @param points the Points to draw
     */
    public static void drawPath(Canvas c, double[][] points) {
        c.strokePolyline(points[0], points[1]);
    }
}
