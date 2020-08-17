package org.firstinspires.ftc.teamcode.rework.Modules;

import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;
import org.firstinspires.ftc.teamcode.rework.AutoTools.Waypoint;
import org.firstinspires.ftc.teamcode.rework.MathFunctions;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.Module;
import org.firstinspires.ftc.teamcode.rework.Robot;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.rework.MathFunctions.*;

public class PathModule implements Module {

    Robot robot;

    public ArrayList<Waypoint> path = new ArrayList<Waypoint>();
    private double followRadius = 15;

    public PathModule(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {
        findTargetPoint();
    }

    public void findTargetPoint(){

        Point followPoint = new Point();

        for (int i = 0; i < path.size() - 1; i++){
            Point start = path.get(i).toPoint();
            Point end = path.get(i+1).toPoint();

            ArrayList<Point> intersections = lineCircleIntersection(new Point(robot.odometryModule.worldX, robot.odometryModule.worldY), followRadius, start, end);

            double nearestAngle = Double.MAX_VALUE;
            for (Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.x - robot.odometryModule.worldX, thisIntersection.y - robot.odometryModule.worldY);
                double deltaAngle = Math.abs(angleWrap2(angle - robot.odometryModule.worldAngleRad));

                if (deltaAngle < nearestAngle){
                    nearestAngle = deltaAngle;
                    followPoint.x = thisIntersection.x;
                    followPoint.y = thisIntersection.y;
                }
            }
        }

        robot.targetModule.targetPointX = followPoint.x;
        robot.targetModule.targetPointY = followPoint.y;
    }
}
