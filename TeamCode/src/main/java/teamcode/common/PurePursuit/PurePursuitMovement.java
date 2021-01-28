package teamcode.common.PurePursuit;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;

import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.Point;
import teamcode.common.PurePursuit.CurvePoint;
import teamcode.common.PurePursuit.Line;
import teamcode.common.PurePursuit.MathFunctions;
import teamcode.common.Vector2D;

import static java.lang.Math.*;

public class PurePursuitMovement {
    private static final double Y_POINT_TOLERANCE_INTERSECT = 0.003;
    private static final double X_POINT_TOLERANCE_INTERSECT = 0.003;


    private static String debugConglomerate = "";
    public boolean isActive;

    private ArrayList<Point> visitedPoints = new ArrayList();

    private Localizer robot;

    private final double ROBOT_RADIUS_CENTIMETERS = 9 * 2.54;
    private int currentRobotIndex;
    private double distanceTolerance;

    public PurePursuitMovement(Localizer robot) {
        this.robot = robot;
        isActive = true;
        currentRobotIndex = 0;
    }


    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> path, Point robotLocation, double followRadius)  {
        CurvePoint followMe = new CurvePoint(path.get(0));
        for (int i = 0; i < path.size() - 1; i++) {
            CurvePoint startLine = path.get(i);
            CurvePoint endLine = path.get(i + 1);
            //ArrayList<Point> intersections = getCircleLineIntersectionPoint(startLine.toPoint(), endLine.toPoint(),robotLocation,followRadius);
            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());
            double closestAngle = 100000000;
            for (Point currentIntersection : intersections) {
                double angle = atan2(currentIntersection.y - robot.getCurrentPosition().y, currentIntersection.x - robot.getCurrentPosition().x);
                double deltaAngle = abs(MathFunctions.angleWrap(angle - robot.getGlobalRads()));
                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(currentIntersection);
                }

            }
        }

        return followMe;
    }


    //TODO implement a method to stop oscillation at the end of the path
    //done
    //TODO using a list of path points, figure out where I am in the path using perpendicular lines. Timestamp 15:00 in the GF tutorial.
    //Implemented however I need to test its logic further
    //further tested and completed
    //TODO need to implement SlowDownTurn effect
    //may be unnecessary? Keep an eye on this for Spline Entanglement
    //TODO need to complete oscillation by extension of the line by the followDistance + 5% (approx)
    //done, however needs to be tuned
    //TODO need to add the braking logic
    //done
    //TODO need to tackle Spline Entanglement
    //May have been fixed?!
    //think it has
    //TODO need to clean this file up
    //done

    private boolean robotIsNearPoint(CurvePoint current) {
        Point currentPoint = current.toPoint();
        return sqrt(pow(currentPoint.x - robot.getCurrentPosition().x, 2) + pow(currentPoint.y - robot.getCurrentPosition().y, 2)) <= current.followDistance;
    }


    //To be implemented into the new Drive System for league 3

    private final double SLOW_DOWN_END_OF_PATH = 0.402; //TODO Calibrated Constant

    //calibrated to sim atm
    public ArrayList<CurvePoint> initPath(ArrayList<CurvePoint> allPoints) {
        CurvePoint secondToLastPoint = allPoints.get(allPoints.size() - 2);
        CurvePoint lastPoint = allPoints.get(allPoints.size() - 1);
        Line finalLine = new Line(secondToLastPoint.toPoint(), lastPoint.toPoint());
        Vector2D velocity = new Vector2D(lastPoint.moveSpeed * cos(finalLine.getAngleRads()), lastPoint.moveSpeed * sin(finalLine.getAngleRads()));
        double finalFollowDistance = SLOW_DOWN_END_OF_PATH * velocity.magnitude() * allPoints.get(allPoints.size() - 1).followDistance;
        allPoints.add(new CurvePoint(lastPoint.x + finalFollowDistance * cos(finalLine.getAngleRads()), lastPoint.y + finalFollowDistance * sin(finalLine.getAngleRads()), lastPoint.moveSpeed, lastPoint.turnSpeed, lastPoint.followDistance, lastPoint.slowDownTurnRads, lastPoint.slowDownTurnAmount));
        isActive = true;
        return allPoints;
    }

    /**
     * @param allPoints   path the robot is pursuing
     * @param followAngle angle the robot should stick to in degrees
     */
    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle)  {
        CurvePoint followMe;
        currentRobotIndex = findPositionInPath(allPoints, robot.getCurrentPosition());
        followMe = getFollowPointPath(allPoints, new Point(robot.getCurrentPosition().x, robot.getCurrentPosition().y), allPoints.get(currentRobotIndex).followDistance);
        if(robotIsNearPoint(allPoints.get(allPoints.size() - 1))) {
            brake();
            return;
        }
        distanceTolerance = allPoints.get(currentRobotIndex).followDistance;
        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    private int findPositionInPath(ArrayList<CurvePoint> allPoints, Point currentPosition) {
        ArrayList<int[]> lineIndexes = new ArrayList<>();
        ArrayList<Point> validIntersections = new ArrayList<>();
        for (int i = currentRobotIndex; i < allPoints.size() - 1; i++) {
            Point firstPoint = allPoints.get(i).toPoint();
            Point nextPoint = allPoints.get(i + 1).toPoint();
            Line current = new Line(firstPoint, nextPoint);
            Double perpendicularSlope;
            Line potentialIntersection;
            if (current.slope == null) {
                perpendicularSlope = 0.0;
                potentialIntersection = new Line(new Point(currentPosition.x - 9, currentPosition.y), new Point(currentPosition.x + 9, currentPosition.y + perpendicularSlope));
            } else if (current.slope == 0.0) {
                potentialIntersection = new Line(new Point(currentPosition.x, currentPosition.y - 9), new Point(currentPosition.x, currentPosition.y + 9));
            } else {
                perpendicularSlope = -1.0 / current.slope;
                potentialIntersection = new Line(currentPosition, new Point(currentPosition.x + 1, currentPosition.y + perpendicularSlope));
            }

            Point intersection = current.segmentIntersection(potentialIntersection);
            if (intersection != null) {
                //it is a valid intersection point, within the domain of the segment
                lineIndexes.add(new int[]{i, i + 1});
                validIntersections.add(intersection);
            }
        }
        double minDistance = Double.MAX_VALUE;
        int minDistanceIndex = currentRobotIndex;
        for (int i = 0; i < validIntersections.size(); i++) {
            double currentDistance = currentPosition.getDistance(validIntersections.get(i));
            if (currentDistance < minDistance) {
                minDistance = min(currentDistance, minDistance);
                minDistanceIndex = lineIndexes.get(i)[1];
            }
        }
        return minDistanceIndex;
    }

    private void brake() {
        MovementVars.movementX = 0;
        MovementVars.movementY = 0;
        MovementVars.movementTurn = 0;
        isActive = false;
    }

    /**
     * moves to any point on the field with a defined coordinate plane
     *
     * @param x              The X coordinate to be moved to
     * @param y              the Y coordinate to be moved to
     * @param power          full power the robot should be at
     * @param preferredAngle angle the robot should be moving at in degrees
     */
    public void goToPosition(double x, double y, double power, double preferredAngle, double turnPower) {
        preferredAngle = toRadians(preferredAngle);

        double deltaX = x - robot.getCurrentPosition().x;
        //System.out.println("X: " + deltaX);
        double deltaY = y - robot.getCurrentPosition().y;
        //System.out.println("Y: " + deltaY);
        double distanceTravelled = hypot(deltaX, deltaY);
        //change in robots position throughout this function
        double absoluteAngle = atan2(deltaY, deltaX);
        //System.out.println(absoluteAngle);
        //angle of motion
        double relativeAngle = MathFunctions.angleWrap(absoluteAngle - (robot.getGlobalRads()));
        //change in angle
        //change in the robots position
        double relativeDistanceX = distanceTravelled * cos(relativeAngle);
        double relativeDistanceY = distanceTravelled * sin(relativeAngle);
        //relative distance the robot is travelling
        double powerX = relativeDistanceX / (abs(relativeDistanceX) + abs(relativeDistanceY));
        double powerY = relativeDistanceY / (abs(relativeDistanceX) + abs(relativeDistanceY));
        //power calculations
        MovementVars.movementX = powerX * power;
        MovementVars.movementY = powerY * power;
        //assigning power to driveTrain
        double relativeTurnAngle = relativeAngle + preferredAngle;
        MovementVars.movementTurn = clip(relativeTurnAngle / toRadians(30), -1, 1) * turnPower;
        if(distanceTravelled < distanceTolerance){
            MovementVars.movementTurn = 0;
        }
    }


    private ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        //(mx+b)^2 = r^2 + x^2
        if (abs(linePoint1.y - linePoint2.y) < Y_POINT_TOLERANCE_INTERSECT) {
            linePoint1.y = linePoint2.y + Y_POINT_TOLERANCE_INTERSECT;
        }
        if (abs(linePoint1.x - linePoint2.x) < X_POINT_TOLERANCE_INTERSECT) {
            linePoint1.x = linePoint2.x + X_POINT_TOLERANCE_INTERSECT;
        }
        double slope1 = linePoint2.slope(linePoint1);


        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;
        double x2 = linePoint2.x - circleCenter.x;
        double y2 = linePoint2.y - circleCenter.y;
        //Defines everything in terms of the Circle center by offsetting it which is added back later, this simplifies the math

        double a = pow(slope1, 2) + 1.0;
        //double b = 2 * line.slope * line.yIntercept;
        double b = (2 * slope1 * y1) - (2 * pow(slope1, 2) * x1);

        //double c = pow((linePoint2.y - linePoint2.x * slope1), 2) + pow(radius, 2);
        double c = (pow(slope1, 2) * pow(x1, 2)) - (2.0 * slope1 * y1 * x1) + pow(y1, 2) - pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList();
        double discriminant = sqrt((pow(b, 2) - 4.0 * a * c));
        //System.out.println(discriminant);
        double xRoot1 = (-b + discriminant) / (2 * a);
        double xRoot2 = (-b - discriminant) / (2 * a);

        double yRoot1 = slope1 * (xRoot1 - x1) + y1;
        double yRoot2 = slope1 * (xRoot2 - x1) + y1;

        //undo the offset from above
        xRoot1 += circleCenter.x;
        yRoot1 += circleCenter.y;
        xRoot2 += circleCenter.x;
        yRoot2 += circleCenter.y;

        double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
        double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
        if (xRoot1 > minX && xRoot1 < maxX) {
            allPoints.add(new Point(xRoot1, yRoot1));
        }
        if (xRoot2 > minX && xRoot2 < maxX) {
            allPoints.add(new Point(xRoot2, yRoot2));
        }
        return allPoints;
    }


    public double clip(double number, double min, double max) {
        if (number < min) return min;
        if (number > max) return max;
        return number;
    }


    /**
     * to be used FOR DEBUGGING PURPOSES
     *
     * @return the position of the robot on the path
     */
    public int getCurrentRobotIndex() {
        return currentRobotIndex;
    }


}
