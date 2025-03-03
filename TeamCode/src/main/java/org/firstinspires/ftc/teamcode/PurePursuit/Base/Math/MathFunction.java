package org.firstinspires.ftc.teamcode.PurePursuit.Base.Math;

import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Vector;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.WayPoint;
import org.opencv.core.Point;

public class MathFunction {

    /**
     * Makes sure the range of the angle is within the limits of 180, -180 degrees
     * @param angle
     * @return
     */
    public static double calculateAngleUnwrap(double angle) {
        while (angle < -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }

        return angle;
    }

    public static double dot(double[] a, double[] b) {
        if (a.length != b.length) {
            throw new IllegalArgumentException("Vectors must be the same length");
        }
        double sum = 0;
        for (int i = 0; i < a.length; i++) {
            sum += a[i] * b[i];
        }
        return sum;
    }

    /**
     * Finds the x, y coordinates of the intersections between the circle and the current segment
     * @param state
     * @param radius
     * @param start
     * @param end
     * @return
     */
    public static Vector calculateCircleIntersection(Vector state, double radius,
                                                                     Vector start, Vector end) {

        double[] d = {end.x - start.x, end.y - start.y};

        // Calculate the differences in x, y between the robot's position and the segment's start
        double[] f = {start.x - state.x, start.y - state.y};

        // Coefficients for the quadratic equation (a*t^2 + b*t + c = 0)
        double a = dot(d, d);
        double b = 2 * dot(f, d);
        double c = dot(f, f) - radius * radius;

        // Calculate the discriminant to check if the circle intersects with the segment
        double discriminant = b * b - 4 * a * c;

        // If discriminant is negative, no intersection; emergency route
        if (discriminant < 0) {
            return null;
        } else {
            // Calculate the two possible values of t where intersections occur
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b + discriminant) / (2 * a);
            double t2 = (-b - discriminant) / (2 * a);
            double t = Math.max(t1, t2);

            if (t < 0.0 || t > 1.0) {
                return null;
            }

            double lx = start.x + t * d[0];
            double ly = start.y + t * d[1];

            return new Vector(lx, ly);
        }
    }

    /**
     * Stopping condition for the robot's position
     * @param currentPose
     * @param targetPoint
     * @return
     */
    public static boolean atTarget(Pose currentPose, WayPoint targetPoint) {
        return Math.abs(targetPoint.pose.x - currentPose.x) <= targetPoint.threshold[0]
            && Math.abs(targetPoint.pose.y - currentPose.y) <= targetPoint.threshold[0]
            && Math.abs(targetPoint.pose.theta - currentPose.theta) <= targetPoint.threshold[1];
    }
}