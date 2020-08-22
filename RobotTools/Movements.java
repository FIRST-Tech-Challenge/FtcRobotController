package org.firstinspires.ftc.teamcode.rework.RobotTools;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.rework.AutoTools.MathFunctions;
import org.firstinspires.ftc.teamcode.rework.AutoTools.PIDController;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;
import org.firstinspires.ftc.teamcode.rework.Robot;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.rework.AutoTools.MathFunctions.*;

public class Movements {

    Robot robot;

    // constants
    private final double distanceThreshold = 0.25;
    private final double angleThreshold = Math.toRadians(2);
    private final double followRadius = 15;

    // states
    private boolean isTargetingLastPoint = false;
    private PIDController pidController;

    // settings
    private double direction = 0;
    private double angleLockHeading = 0;
    private boolean willAngleLock = false;

    public Movements(Robot robot) {
        this.robot = robot;
        pidController = new PIDController(0.05,0.00000001,0,robot);
    }

    /**
     * The index of the point that the robot is currently following in the path.
     */
    private int pathIndex = 0;

    public void pathFollow(ArrayList<Waypoint> path, double direction, double moveSpeed, double turnSpeed, boolean willAngleLock, double angleLockHeading) {

        pathIndex = 0; // Reset pathIndex
        this.direction = direction;
        this.willAngleLock = willAngleLock;
        this.angleLockHeading = angleLockHeading;
        isTargetingLastPoint = false;

        while (robot.isOpModeActive()) {
            Point robotPoint = new Point(robot.odometryModule.worldX, robot.odometryModule.worldY);
            double robotHeading = robot.odometryModule.worldAngleRad;

            Point clippedPoint = clipToPath(path, robotPoint);

            Point targetPoint = findTarget(path, clippedPoint, robotHeading);

            setMovementsToTarget(targetPoint, moveSpeed, turnSpeed);

            if (isDone(path, robotPoint, robotHeading)) {
                robot.drivetrainModule.xMovement = 0;
                robot.drivetrainModule.yMovement = 0;
                robot.drivetrainModule.turnMovement = 0;
                robot.telemetryDump.addData("DEFNLIALG",0);
                return;
            }
        }
    }

    private Point clipToPath(ArrayList<Waypoint> path, Point center) {
        Point clipped = new Point();

        double nearestClipDist = Double.MAX_VALUE;
        int clippedIndex = pathIndex;

        // only checks the current line and the next line (no skipping)
        for (int i = pathIndex; i < Math.min(path.size() - 1, pathIndex + 2); i++) {
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

        pathIndex = clippedIndex;
        robot.telemetryDump.addData("pathIndex", pathIndex);

        return clipped;
    }

    private Point findTarget(ArrayList<Waypoint> path, Point center, double heading) {
        Point followPoint = new Point();

        Point lineStartPoint = path.get(pathIndex).toPoint();
        double distToFirst = Math.hypot(center.x - lineStartPoint.x, center.y - lineStartPoint.y);

        // only look at lines on current index or next index
        for (int i = pathIndex; i < Math.min(path.size() - 1, pathIndex + 2); i++) {
            Point start = path.get(i).toPoint();
            Point end = path.get(i + 1).toPoint();

            ArrayList<Point> intersections = lineCircleIntersection(center, followRadius, start, end);

            double nearestAngle = Double.MAX_VALUE;

            for (Point thisIntersection : intersections) {

                double angle = Math.atan2(thisIntersection.x - center.x, thisIntersection.y - center.y) + direction;
                double deltaAngle = Math.abs(MathFunctions.angleWrap2(angle - heading));
                double thisDistToFirst = Math.hypot(thisIntersection.x - lineStartPoint.x, thisIntersection.y - lineStartPoint.y);

                if (deltaAngle < nearestAngle && thisDistToFirst > distToFirst) {
                    nearestAngle = deltaAngle;
                    followPoint.x = thisIntersection.x;
                    followPoint.y = thisIntersection.y;
                }
            }
        }

        if (Math.hypot(center.x - path.get(path.size() - 1).x, center.y - path.get(path.size() - 1).y) < followRadius * 1.5 && pathIndex == path.size() - 2) {
            followPoint = path.get(path.size() - 1).toPoint();
            isTargetingLastPoint = true;
        }

        return followPoint;
    }

    private void setMovementsToTarget(Point targetPoint, double moveSpeed, double turnSpeed) {
        double distanceToTarget = Math.hypot(targetPoint.x - robot.odometryModule.worldX, targetPoint.y - robot.odometryModule.worldY);
        double absoluteAngleToTarget = Math.atan2(targetPoint.x - robot.odometryModule.worldX, targetPoint.y - robot.odometryModule.worldY);

        double relativeAngleToPoint = absoluteAngleToTarget - robot.odometryModule.worldAngleRad;
        double relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double relativeTurnAngle = angleWrap2(relativeAngleToPoint + direction);
        if (willAngleLock && isTargetingLastPoint){
            relativeTurnAngle = angleWrap2(angleLockHeading - robot.odometryModule.worldAngleRad);
        }

        // adjust vector based on current velocity
        relativeXToPoint -= 0.2 * robot.velocityModule.xVel;
        relativeYToPoint -= 0.2 * robot.velocityModule.yVel;

        double xPower = relativeXToPoint / Math.hypot(relativeYToPoint, relativeXToPoint);
        double yPower = relativeYToPoint / Math.hypot(relativeYToPoint, relativeXToPoint);

        // lol p
        robot.drivetrainModule.xMovement = xPower * moveSpeed;
        robot.drivetrainModule.yMovement = yPower * moveSpeed;
        robot.drivetrainModule.turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (isTargetingLastPoint){
            pidController.PID(new Point(robot.odometryModule.worldX, robot.odometryModule.worldY), targetPoint);
            double scale = pidController.scale;
            robot.drivetrainModule.xMovement *= scale;
            robot.drivetrainModule.yMovement *= scale;
        }
    }

    private boolean isDone(ArrayList<Waypoint> path, Point center, double heading) {
        Point endPoint = path.get(path.size() - 1).toPoint();
        return (Math.hypot(center.x - endPoint.x, center.y - endPoint.y) < distanceThreshold) && (!willAngleLock || Math.abs(angleLockHeading - heading) < angleThreshold) && pathIndex == path.size() - 2;
    }
}
