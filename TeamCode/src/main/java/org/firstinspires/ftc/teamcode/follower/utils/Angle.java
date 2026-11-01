/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.utils;

public class Angle {
    /**
     * This normalizes an angle to be between 0 and 2 pi radians, inclusive.
     * <p>
     * IMPORTANT NOTE: This method operates in radians.
     *
     * @param angleRadians the angle to be normalized.
     * @return returns the normalized angle.
     */
    public static double normalize(double angleRadians) {
        double angle = angleRadians % (2 * Math.PI);
        if (angle < 0) {
            return angle + 2 * Math.PI;
        }
        return angle;
    }

    /**
     * This normalizes an angle to be between -PI and PI radians.
     * <p>
     * IMPORTANT NOTE: This method operates in radians.
     *
     * @param angleRadians the angle to be normalized.
     * @return returns the normalized angle between -PI (inclusive) and PI (exclusive).
     */
    public static double normalizeSigned(double angleRadians) {
        double angle = normalize(angleRadians);
        if (angle >= Math.PI) {
            return angle - 2 * Math.PI;
        }
        return angle;
    }

    /**
     * This returns the smallest angle between two angles. This operates in radians.
     *
     * @param one one of the angles.
     * @param two the other one.
     * @return returns the smallest angle.
     */
    public static double smallestDifference(double one, double two) {
        return Math.min(normalize(one - two), normalize(two - one));
    }

    public static double error(double current, double target) {
        return normalizeSigned(target - current);
    }

    /**
     * This gets the direction to turn between a start heading and an end heading. Positive is left
     * and negative is right. This operates in radians.
     *
     * @return returns the turn direction.
     */
    public static double turnDirection(double startHeading, double endHeading) {
        double normalizedError = normalize(endHeading - startHeading);
        if (normalizedError >= 0 && normalizedError <= Math.PI) {
            return 1; // counter clock wise
        }
        return -1; // clock wise
    }

    public static double turnDirection(double normalizedError) {
        if (normalizedError >= 0 && normalizedError <= Math.PI) {
            return 1; // counter clock wise
        }
        return -1; // clock wise
    }

    public enum Unit {
        DEGREES {
            @Override
            public double toRadians(double heading) {
                return Math.toRadians(heading);
            }

            @Override
            public double fromRadians(double radians) {
                return Math.toDegrees(radians);
            }
        },
        RADIANS {
            @Override
            public double toRadians(double heading) {
                return heading;
            }

            @Override
            public double fromRadians(double radians) {
                return radians;
            }
        };

        public abstract double toRadians(double heading);

        public abstract double fromRadians(double radians);
    }
}
