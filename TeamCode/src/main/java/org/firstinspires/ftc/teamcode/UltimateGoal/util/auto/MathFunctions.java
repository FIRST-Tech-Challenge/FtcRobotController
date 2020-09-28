package org.firstinspires.ftc.teamcode.rework.util.auto;

import java.util.ArrayList;

import static java.lang.Math.pow;

public class MathFunctions {
    /**
     * Wraps angle (in radians) to a value from -pi to pi
     *
     * @param   angle   Angle to be wrapped
     * @return  The wrapped angle, between -pi and pi
     */
    public static double angleWrap(double angle) {
        return angleWrap(angle, 0);
    }

    /**
     * Wraps an angle (in radians) to a value within pi of centerOfWrap.
     *
     * @param   angle          The angle to be wrapped
     * @param   centerOfWrap   The center of the boundary in which the angle can be wrapped to.
     * @return  The wrapped angle, which will lie within pi of the centerOfWrap.
     */
    public static double angleWrap(double angle, double centerOfWrap) {
        if (angle > centerOfWrap + Math.PI) {
            return angleWrap(angle - (2 * Math.PI), centerOfWrap);
        } else if (angle < centerOfWrap - Math.PI) {
            return angleWrap(angle + (2 * Math.PI), centerOfWrap);
        } else {
            return angle;
        }
    }

    /**
     * Returns the distance between two points, uses Math.hypot.
     *
     * @param   a   A point.
     * @param   b   A point.
     * @return  The distance between the points.
     */
    public static double distance(Point a, Point b) {
        return Math.hypot(a.y - b.y, a.x - b.x);
    }

    /**
     * Calculate the distance between a given point and a given line segment represented by two points.
     *
     * This will return the distance from a point to the endpoint of a segment if the point is not
     * within the bounds of the segment.
     *
     * @param   point        The point.
     * @param   linePoint1   A point on the line.
     * @param   linePoint2   Another, different, point on the line.
     * @return  The distance between the point and the line segment determined by the two linePoints.
     */
    public static double lineSegmentPointDistance(Point point, Point linePoint1, Point linePoint2) {
        if (!pointWithinLineSegment(point, linePoint1, linePoint2)) {
            return Math.min(distance(point, linePoint1), distance(point, linePoint2));
        }

        double top = Math.abs((linePoint2.y - linePoint1.y) * point.x - (linePoint2.x - linePoint1.x) * point.y + linePoint2.x * linePoint1.y - linePoint2.y * linePoint1.x);
        double bottom = Math.hypot(linePoint2.y - linePoint1.y, linePoint2.x - linePoint1.x);

        return top / bottom;
    }

    /**
     * Returns whether or not a point is within the bounds of a line segment. This means that the
     * point lies between the endpoints of the segment, rigorously, it lies between the area defined
     * by perpendicular lines at each endpoint of the segment.
     *
     * @param   point        The point.
     * @param   linePoint1   The first endpoint of the line segment.
     * @param   linePoint2   The other endpoint of the line segment.
     * @return  Whether or not the point given lies within bounds of the line segment.
     */
    public static boolean pointWithinLineSegment(Point point, Point linePoint1, Point linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double t1 = Math.atan2(point.y - linePoint1.y, point.x - linePoint1.x);
        double t2 = Math.atan2(point.y - linePoint2.y, point.x - linePoint2.x);
        double tL = Math.atan2(linePoint2.y - linePoint1.y, linePoint2.x - linePoint1.x);

        double a1 = Math.abs(angleWrap(tL - t1));
        double a2 = Math.abs(Math.PI - angleWrap(tL - t2));

        return a1 <= Math.PI / 2 && a2 <= Math.PI / 2;
    }

    /**
     * Given a point and a line (represented by 2 points), this method finds the point on the line
     * closest to the given point.
     *
     * @param   point        The point for which to find the closest point lying on the given line.
     * @param   linePoint1   A point on the line.
     * @param   linePoint2   A different point on the line.
     * @return  The point closest to the given point that lies on the line specified.
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
     * @param   firstLinePoint    A point on the first line.
     * @param   firstLineSlope    The slope of the first line.
     * @param   secondLinePoint   A point on the second line.
     * @param   secondLineSlope   The slope of the second line.
     * @return  The point representing the intersection of the two lines.
     */
    public static Point twoLineIntersectionPoint(Point firstLinePoint, double firstLineSlope, Point secondLinePoint, double secondLineSlope) {
        /* The generalized equation for the intersection of two lines given by y = mx + b is
            x = (b2 - b1) / (m1 - m2), where b2 is the b term for the second line,
            and y can be solved using either line's equation.

            For each line, b = y - mx, where (x, y) is a point on the line. We can substitute this
            into the generalized intersection equation to get the lines below.
         */

        if (firstLineSlope == secondLineSlope) {
            return new Point(); // If the slopes are parallel, return an empty point.
        }

        double x = ((secondLinePoint.y - (secondLineSlope * secondLinePoint.x)) - (firstLinePoint.y - (firstLineSlope * firstLinePoint.x))) // numerator
                / (firstLineSlope - secondLineSlope); // denominator
        double y = (firstLineSlope * (x)) + (firstLinePoint.y - (firstLineSlope * firstLinePoint.x)); // Using y = mx + b

        return new Point(x, y);
    }

    /**
     * Solves for the real roots of a quadratic equation in the form of ax^2 + bx + c.
     *
     * Will return null values if there are no real solution, or 1 value followed by null if there
     * is only one real solution.
     *
     * @param   a   coefficient of squared term
     * @param   b   coefficient of linear term
     * @param   c   constant
     * @return  double[2] with the roots of the quadratic.
     */
    public static double[] solveQuadratic(double a, double b, double c) {
        /* The equation for the roots of a quadratic equation is: (-b +/- sqrt(b^2-4ac)) / 2a
         */
        double[] roots = new double[2];
        double discriminant = (b * b) - (4 * a * c);

        if (discriminant < 0) {
            return roots;
        } else if (discriminant != 0) {
            roots[1] = (-b - Math.sqrt(discriminant)) / (2 * a);
        }
        roots[0] = (-b + Math.sqrt(discriminant)) / (2 * a);

        return roots;
    }

    /**
     * Returns all the intersections between a segment of a line and a circle.
     *
     * @param circleCenter   The center of the circle.
     * @param radius         The radius of the circle.
     * @param linePoint1     An end point of the segment.
     * @param linePoint2     The other end point of the segment.
     * @return An ArrayList of points representing the intersection between the circle and the line segment.
     */
    public static ArrayList<Point> lineSegmentCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        ArrayList<Point> solutions = new ArrayList<>();

        //make sure the points don't exactly line up so the slopes work
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        //calculate the slope of the line
        double lineSlope = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + pow(lineSlope, 2);

        //shift one of the line's points so it is relative to the circle
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * lineSlope * y1) - (2.0 * pow(lineSlope, 2) * x1);
        double quadraticC = ((pow(lineSlope, 2) * pow(x1, 2)) - (2.0 * y1 * lineSlope * x1) + pow(y1, 2) - pow(radius, 2));

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
                solutions.add(new Point(xRoot1, yRoot1));
            }

            //do the same for the other root
            double xRoot2 = (-quadraticB - Math.sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot2 = lineSlope * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot2 > minX && xRoot2 < maxX) {
                solutions.add(new Point(xRoot2, yRoot2));
            }

        } catch (Exception e) {
            //if there are no roots
        }
        return solutions;
    }
}
