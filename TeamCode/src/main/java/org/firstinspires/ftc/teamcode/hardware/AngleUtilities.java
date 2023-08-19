package org.firstinspires.ftc.teamcode.Utility;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AngleUtilities {

    /**
     * Normalizes an angle to be between -180 and 179 degrees
     *
     * @param angle angle to be normalized
     * @return an angle to be between -180 and 179 degrees
     */
    public static double getNormalizedAngle(double angle) {
        return AngleUnit.normalizeDegrees(angle);
    }

    public static double radiansDegreesTranslation(double radians) {
        double degrees = radians * 180 / Math.PI;
        return degrees;
    }

    /**
     * Normalizes an angle to be between 0 and 359 degrees
     *
     * @param angle angle to be normalized
     * @return an angle between 0 and 359 degrees
     */
    public static double getPositiveNormalizedAngle(double angle) {
        return (getNormalizedAngle(angle) + 360) % 360;
    }

    public static double getRadius(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public static double getAngle(double x, double y) {

        double angleRad = Math.atan2(y, x);
        double angleDegrees = AngleUtilities.radiansDegreesTranslation(angleRad);

        return AngleUtilities.getNormalizedAngle(angleDegrees);
    }
}