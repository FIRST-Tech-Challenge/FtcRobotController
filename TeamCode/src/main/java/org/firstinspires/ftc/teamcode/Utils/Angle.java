package org.firstinspires.ftc.teamcode.Utils;

public final class Angle {

    public static double wrapAngle0_360(final double theta) {
        return theta % 360;
    }
    // convert 0-360 iRadians

    public static double wrapAnglePlusMinus_180(final double theta) {
        final double wrapped = theta % 360; // Getting the angle smallest form (not exceeding a
        // full turn).
// convert -180-180 in Radians
        // Adding or subtracting two pi to fit the range of +/- pi.
        if (wrapped > 180) {
            return wrapped - 360;
        } else if (wrapped < -180) {
            return wrapped + 360;
        } else {
            return wrapped;
        }
    }

    // Functions to convert between degrees and radians:
    public static float degToRad(final float theta) {
        return (float) Math.toRadians(theta);
    } //float

    public static float radToDeg(final float theta) {
        return (float) Math.toDegrees(theta);
    } //float

    public static float projectBetweenPlanes(final float theta, final float alpha) {
        if (alpha < 0) {
            return (float) Math.atan(Math.tan(theta) / Math.cos(alpha));
        } else {
            return (float) Math.atan(Math.tan(theta) * Math.cos(alpha));
        }
    }
    // alpha is the angle between the planes.
    // For more information check:
    // https://math.stackexchange.com/questions/2207665/projecting-an-angle-from-one-plane-to-another-plane
}

