package org.firstinspires.ftc.teamcode.rework.AutoTools;

import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;

import java.util.ArrayList;
import java.util.Vector;

import static java.lang.Math.pow;

public class MathFunctions {

    /**
     * Wraps angle (in radians) to a value from 0 to 2*pi
     *
     * @param angle Angle to be wrapped
     * @return The wrapped angle
     */
    public static double angleWrap(double angle) {
        return angle % (2 * Math.PI);
    }

    /**
     * Wraps an angle (in radians) to a value within pi of centerOfWrap.
     *
     * @param angle The angle to be wrapped
     * @param centerOfWrap The center of the boundary in which the angle can be wrapped to.
     * @return The wrapped angle, which will lie within pi of the centerOfWrap.
     */
    public static double angleWrap(double angle, double centerOfWrap) {
       if (Math.abs(angle) > centerOfWrap + Math.PI) {
           return angleWrap(angle - (2 * Math.PI), centerOfWrap);
       } else if (Math.abs(angle) < centerOfWrap - Math.PI) {
           return angleWrap(angle + (2 * Math.PI), centerOfWrap);
       } else {
           return angle;
       }
    }

    public static double linePointDistance(Point point, Point linePoint1, Point linePoint2) {
        double top = Math.abs((linePoint2.y - linePoint1.y) * point.x - (linePoint2.x - linePoint1.x) * point.y + linePoint2.x * linePoint1.y - linePoint2.y * linePoint1.x);
        double bottom = Math.hypot(linePoint2.y - linePoint1.y, linePoint2.x - linePoint1.x);

        return top / bottom;
    }

    public static Point closestPointOnLineToPoint(Point point, Point linePoint1, Point linePoint2) {

        double m;

        if (linePoint2.x == linePoint1.x) {
            return new Point(linePoint1.x, point.y);
        } else if (linePoint2.y == linePoint1.y) {
            return new Point(point.x, linePoint1.y);
        } else {
            m = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        }

        return twoLineIntersectionPoint(linePoint1, m, point, -1 / m);
    }

    public static Point twoLineIntersectionPoint(Point firstLinePoint, double firstLineM, Point secondLinePoint, double secondLineM) {
        double x = (firstLineM * firstLinePoint.x - firstLinePoint.y - secondLineM * secondLinePoint.x + secondLinePoint.y) / (firstLineM - secondLineM);
        double y = firstLineM * (x - firstLinePoint.x) + secondLinePoint.y;

        return new Point(x, y);
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        //make sure the points don't exactly line up so the slopes work
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        //calculate the slope of the line
        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + pow(m1, 2);

        //shift one of the line's points so it is relative to the circle
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1, 2) * x1);
        double quadraticC = ((pow(m1, 2) * pow(x1, 2)) - (2.0 * y1 * m1 * x1) + pow(y1, 2) - pow(radius, 2));


        ArrayList<Point> allPoints = new ArrayList<>();
        try {
            // solve roots
            double xRoot1 = (-quadraticB + Math.sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            //add back in translations
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            //within range
            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            //do the same for the other root
            double xRoot2 = (-quadraticB - Math.sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }

        } catch (Exception e) {
            //if there are no roots
        }
        return allPoints;
    }
}
