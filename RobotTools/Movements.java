package org.firstinspires.ftc.teamcode.rework.RobotTools;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;
import org.firstinspires.ftc.teamcode.rework.Robot;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.rework.AutoTools.MathFunctions.angleWrap2;
import static org.firstinspires.ftc.teamcode.rework.AutoTools.MathFunctions.*;

public class Movements {

    Robot robot;
    private double endTreshold = 0.25;
    private double followRadius = 15;

    public Movements(Robot robot) {
        this.robot = robot;
    }

    private int index = 0;

    public void pathFollow(ArrayList<Waypoint> path, double moveSpeed, double turnSpeed) {

        index = 0;

        while (robot.isOpModeActive()) {
            Point robotPoint = new Point(robot.odometryModule.worldX, robot.odometryModule.worldY);

            Point clippedPoint = clipToPath(path, robotPoint);

            Point targetPoint = findTarget(path, clippedPoint, followRadius, robot.odometryModule.worldAngleRad);

            setMovementsToTarget(targetPoint, moveSpeed, turnSpeed);

            robot.telemetryDump.addData("index ", index);

            if (isDone(path, robotPoint)) {
                robot.drivetrainModule.xMovement = 0;
                robot.drivetrainModule.yMovement = 0;
                robot.drivetrainModule.turnMovement = 0;
                return;
            }
        }
    }

    private Point clipToPath(ArrayList<Waypoint> path, Point center) {
        Point clipped = new Point();

        double nearestClipDist = Double.MAX_VALUE;
        int clippedIndex = index;

        // only checks the current line and the next line (no skipping)
        for (int i = index; i < Math.min(path.size() - 1, index + 2); i++) {
            Point start = path.get(i).toPoint();
            Point end = path.get(i + 1).toPoint();

            double thisClipDist = linePointDistance(center, start, end);

            // if this clip distance is record low set the clip point to the clip point set the clippedIndex to index so later we can update the index we are at
            if (thisClipDist < nearestClipDist) {
                nearestClipDist = thisClipDist;
                clipped = closestPointOnLineToPoint(center, start, end);
                clippedIndex = i;
            }
        }

        index = clippedIndex;

        return clipped;
    }

    private Point findTarget(ArrayList<Waypoint> path, Point center, double followRadius, double heading) {
        Point followPoint = new Point();

        // only look at lines on current index or next index
        for (int i = index; i < Math.min(path.size() - 1, index + 2); i++) {
            Point start = path.get(i).toPoint();
            Point end = path.get(i + 1).toPoint();

            ArrayList<Point> intersections = lineCircleIntersection(center, followRadius, start, end);

            double nearestAngle = Double.MAX_VALUE;
            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.x - center.x, thisIntersection.y - center.y);
                double deltaAngle = Math.abs(angleWrap2(angle - heading));

                if (deltaAngle < nearestAngle) {
                    nearestAngle = deltaAngle;
                    followPoint.x = thisIntersection.x;
                    followPoint.y = thisIntersection.y;
                }
            }
        }

        if (Math.hypot(center.x - path.get(path.size() - 1).x, center.y - path.get(path.size() - 1).y) < followRadius * 1.5 && index == path.size() - 2) {
            followPoint = path.get(path.size() - 1).toPoint();
        }

        return followPoint;
    }

    private void setMovementsToTarget(Point targetPoint, double moveSpeed, double turnSpeed) {
        double distanceToTarget = Math.hypot(targetPoint.x - robot.odometryModule.worldX, targetPoint.y - robot.odometryModule.worldY);
        double absoluteAngleToTarget = Math.atan2(targetPoint.x - robot.odometryModule.worldX, targetPoint.y - robot.odometryModule.worldY);

        double relativeAngleToPoint = absoluteAngleToTarget - robot.odometryModule.worldAngleRad;
        double relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeTurnAngle = angleWrap2(relativeAngleToPoint);

        // adjust vector based on current velocity
        relativeXToPoint -= 0.2 * robot.velocityModule.xVel;
        relativeYToPoint -= 0.2 * robot.velocityModule.yVel;

        double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

        robot.drivetrainModule.xMovement = xPower * moveSpeed;
        robot.drivetrainModule.yMovement = yPower * moveSpeed;
        robot.drivetrainModule.turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
    }

    private boolean isDone(ArrayList<Waypoint> path, Point center) {
        Point endPoint = path.get(path.size() - 1).toPoint();

        return (Math.hypot(center.x - endPoint.x, center.y - endPoint.y) < endTreshold) && index == path.size() - 2;
    }
}
