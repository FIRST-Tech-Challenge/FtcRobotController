package org.firstinspires.ftc.teamcode.rework;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.rework.MathFunctions.angleWrap2;
import static org.firstinspires.ftc.teamcode.rework.MathFunctions.lineCircleIntersection;

public class Movements {

    Robot robot;
    private double endTreshold = 0.125;
    private double followRadius = 15;

    public Movements(Robot robot){
        this.robot = robot;
    }

    public void pathFollow(ArrayList<Waypoint> path, double moveSpeed, double turnSpeed){
        while (robot.isOpModeActive()){
            Point followPoint = findTarget(path, followRadius, new Point(robot.odometryModule.worldX, robot.odometryModule.worldY), robot.odometryModule.worldAngleRad);

            if (setMovementsToTarget(followPoint, moveSpeed, turnSpeed)){
                break;
            }
        }
    }

    private Point findTarget(ArrayList<Waypoint> path, double followRadius, Point center, double heading){

        Point followPoint = new Point();

        for (int i = 0; i < path.size() - 1; i++) {
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

        if (Math.hypot(center.x - path.get(path.size() - 1).x, center.y - path.get(path.size() - 1).y) < followRadius * 1.5){
            followPoint = path.get(path.size()-1).toPoint();
        }

        return followPoint;
    }

    // true if close to target
    private boolean setMovementsToTarget(Point targetPoint, double moveSpeed, double turnSpeed){
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

        if (distanceToTarget < endTreshold){
            robot.drivetrainModule.xMovement = 0;
            robot.drivetrainModule.yMovement = 0;
            robot.drivetrainModule.turnMovement = 0;
        } else {
            robot.drivetrainModule.xMovement = xPower * moveSpeed;
            robot.drivetrainModule.yMovement = yPower * moveSpeed;
            robot.drivetrainModule.turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
        }

        return distanceToTarget < endTreshold;
    }
}
