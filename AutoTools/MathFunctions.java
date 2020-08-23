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

    public static double angleWrap2(double angle) {
       if (angle > Math.PI) {
           return angleWrap2(angle - (2 * Math.PI));
       } else if (angle < - Math.PI) {
           return angleWrap2(angle + (2 * Math.PI));
       } else {
           return angle;
       }
    }

    /**
     * Calculate the distance between a given point and a given line, represented by two points.
     *
     * @param point The point.
     * @param linePoint1 A point on the line.
     * @param linePoint2 Another, different, point on the line.
     * @return The distance between the point and the line determined by the two linePoints.
     */
    public static double linePointDistance(Point point, Point linePoint1, Point linePoint2) {
        /* From ax + by + c = 0, the equation for the distance between a point (x0, y0) and that
            line is ((a * x0) + (b * y0) + c)/sqrt(a^2 + b^2). This can be derived geometrically
            or with some algebra.

            The general standard form (ax + by + c = 0) equation of a line with points (x1,y1) and
            (x2,y2) is (y2 - y1)x - (x2-x1)y + x2y1 - y2x1 = 0 (derived by plugging our points into
            slope-intercept and rearranging). So, in our case, a = (y2 - y1), b = -(x2 - x1),
            and c = x2y1 - y2x1.

            We use these a, b, and c in our distance equation from the first paragraph to get the
            lines below.
         */

        double top = Math.abs((linePoint2.y - linePoint1.y) * point.x // a
                - (linePoint2.x - linePoint1.x) * point.y // b
                + linePoint2.x * linePoint1.y - linePoint2.y * linePoint1.x); // c

        double bottom = Math.hypot(linePoint2.y - linePoint1.y, linePoint2.x - linePoint1.x);

        return top / bottom;
    }

    /**
     * Given a point and a line (represented by 2 points), this method finds the point on the line
     * closest to the given point.
     *
     * @param point The point for which to find the closest point lying on the given line.
     * @param linePoint1 A point on the line.
     * @param linePoint2 A different point on the line.
     * @return The point closest to the given point that lies on the line specified.
     */
    public static Point closestPointOnLineToPoint(Point point, Point linePoint1, Point linePoint2) {
        double lineSlope;

        if (linePoint2.x == linePoint1.x) { // If line is vertical
            return new Point(linePoint1.x, point.y);
        } else if (linePoint2.y == linePoint1.y) { // If line is horizontal
            return new Point(point.x, linePoint1.y);
        } else { // Oblique line
            lineSlope = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        }

        /* The closest point to the given point on the given line will be the intersection between
            the given line and the line perpendicular to that line that passes through the point.
         */
        return twoLineIntersectionPoint(linePoint1, lineSlope, point, -1 / lineSlope);
    }

    /**
     * Returns the intersection between two lines.
     *
     * @param firstLinePoint A point on the first line.
     * @param firstLineSlope The slope of the first line.
     * @param secondLinePoint A point on the second line.
     * @param secondLineSlope The slope of the second line.
     * @return The point representing the intersection of the two lines.
     */
    public static Point twoLineIntersectionPoint(Point firstLinePoint, double firstLineSlope, Point secondLinePoint, double secondLineSlope) {
        /* The generalized equation for the intersection of two lines given by y = mx + b is
            x = (b2 - b1) / (m1 - m2), where b2 is the b term for the second line,
            and y can be solved using either line's equation.

            For each line, b = y - mx, where (x, y) is a point on the line. We can substitute this
            into the generalized intersection equation to get the lines below.
         */

        double x = ((secondLinePoint.y - (secondLineSlope * secondLinePoint.x)) - (firstLinePoint.y - (firstLineSlope * firstLinePoint.x))) // numerator
                / (firstLineSlope - secondLineSlope); // denominator
        double y = (firstLineSlope * (x)) + (firstLinePoint.y - (firstLineSlope * firstLinePoint.x)); // Using y = mx + b

        return new Point(x, y);
    }

    /**
     * Returns all the intersections between a line and a circle.
     *
     * @param circleCenter The center of the circle.
     * @param radius The radius of the circle.
     * @param linePoint1 A point on the line.
     * @param linePoint2 A different point on the line.
     * @return An ArrayList of points representing the intersection between the circle and the line.
     */
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        // make sure the points don't exactly line up so the slopes work
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        // calculate the slope of the line
        double lineSlope = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + pow(lineSlope, 2);

        //shift one of the line's points so it is relative to the circle
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * lineSlope * y1) - (2.0 * pow(lineSlope, 2) * x1);
        double quadraticC = ((pow(lineSlope, 2) * pow(x1, 2)) - (2.0 * y1 * lineSlope * x1) + pow(y1, 2) - pow(radius, 2));

        ArrayList<Point> allPoints = new ArrayList<>();
        try {
            // solve roots
            double xRoot1 = (-quadraticB + Math.sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot1 = lineSlope * (xRoot1 - x1) + y1;

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
            double yRoot2 = lineSlope * (xRoot2 - x1) + y1;

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
