package org.firstinspires.ftc.teamcode.rework.RobotTools;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.rework.AutoTools.MathFunctions;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.TelemetryProvider;
import org.firstinspires.ftc.teamcode.rework.Robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.rework.AutoTools.MathFunctions.*;

public class Movements implements TelemetryProvider {

    Robot robot;

    private boolean isFileDump;

    Point clippedPoint;
    Point targetPoint;
    Point adjustedTargetPoint;


    // constants
    private final double distanceThreshold = 0.5;
    private final double angleThreshold = Math.toRadians(2);
    private final double followRadius = 15;
    private final double slipFactor = 0;

    public int currentTrip;

    // states
    private boolean isTargetingLastPoint = false;

    // settings
    private double direction = 0;
    private double angleLockHeading = 0;
    private boolean willAngleLock = false;

    double relativeAngleToPoint, relativeXToPoint, relativeYToPoint;

    public Movements(Robot robot, boolean isFileDump) {
        TelemetryDump.registerProvider(this);
        this.robot = robot;
        currentTrip = 1;
        this.isFileDump = isFileDump;
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

        pathFileDump(path);

        while (robot.isOpModeActive()) {
            Point robotPoint = new Point(robot.odometryModule.worldX, robot.odometryModule.worldY);

            double robotHeading = robot.odometryModule.worldAngleRad;

            clippedPoint = clipToPath(path, robotPoint);

            targetPoint = findTarget(path, clippedPoint, robotHeading);

            adjustedTargetPoint = adjustTargetPoint(targetPoint);

            setMovementsToTarget(adjustedTargetPoint, moveSpeed, turnSpeed);

            fileDump();

            if (isDone(path, robotPoint, robotHeading)) {
                robot.drivetrainModule.xMovement = 0;
                robot.drivetrainModule.yMovement = 0;
                robot.drivetrainModule.turnMovement = 0;
                currentTrip++;
                return;
            }
        }
    }

    private void pathFileDump(ArrayList<Waypoint> path) {
        if (robot.WILL_FILE_DUMP) {
            for (int i = 0; i < path.size(); i++) {
                robot.fileDump.addData(new StringBuilder().append(currentTrip).append("_path.txt").toString(), new StringBuilder().append(path.get(i).x).append(" ").append(path.get(i).y).toString());
            }
        }
    }

    private void fileDump() {
        if (robot.WILL_FILE_DUMP) {
            robot.fileDump.addData(new StringBuilder().append(currentTrip).append("_target.txt").toString(), new StringBuilder().append(adjustedTargetPoint.x).append(" ").append(adjustedTargetPoint.y).toString());
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

    private Point adjustTargetPoint(Point targetPoint) {
        double robotSlipX = slipFactor * robot.velocityModule.xVel;
        double robotSlipY = slipFactor * robot.velocityModule.yVel;

        double slipX = robotSlipX * Math.cos(robot.odometryModule.worldAngleRad) + robotSlipY * Math.sin(robot.odometryModule.worldAngleRad);
        double slipY = robotSlipY * Math.cos(robot.odometryModule.worldAngleRad) - robotSlipX * Math.sin(robot.odometryModule.worldAngleRad);

        return new Point(targetPoint.x - slipX, targetPoint.y - slipY);
    }

    private void setMovementsToTarget(Point targetPoint, double moveSpeed, double turnSpeed) {
        double distanceToTarget = Math.hypot(targetPoint.x - robot.odometryModule.worldX, targetPoint.y - robot.odometryModule.worldY);
        double absoluteAngleToTarget = Math.atan2(targetPoint.x - robot.odometryModule.worldX, targetPoint.y - robot.odometryModule.worldY);

        relativeAngleToPoint = absoluteAngleToTarget - robot.odometryModule.worldAngleRad;
        relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double relativeTurnAngle = angleWrap2(relativeAngleToPoint + direction);
        if (willAngleLock && isTargetingLastPoint) {
            relativeTurnAngle = angleWrap2(angleLockHeading - robot.odometryModule.worldAngleRad);
        }

        double xPower = relativeXToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));
        double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

        // lol p
        robot.drivetrainModule.xMovement = xPower * moveSpeed;
        robot.drivetrainModule.yMovement = yPower * moveSpeed;
        robot.drivetrainModule.turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (isTargetingLastPoint) {
            robot.drivetrainModule.xMovement *= Range.clip(distanceToTarget / followRadius, 0.2, 1);
            robot.drivetrainModule.yMovement *= Range.clip(distanceToTarget / followRadius, 0.2, 1);
        }
    }

    private boolean isDone(ArrayList<Waypoint> path, Point center, double heading) {
        Point endPoint = path.get(path.size() - 1).toPoint();

        return (Math.hypot(center.x - endPoint.x, center.y - endPoint.y) < distanceThreshold) && (!willAngleLock || Math.abs(angleWrap2(angleLockHeading - heading)) < angleThreshold) && pathIndex == path.size() - 2;
    }

    public boolean isFileDump() {
        return isFileDump;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("relativeXToPoint: " + String.valueOf(relativeXToPoint));
        data.add("relativeYToPoint: " + String.valueOf(relativeYToPoint));
        data.add("relativeAngleToPoint: " + String.valueOf(relativeAngleToPoint));
        return data;
    }
}