package org.firstinspires.ftc.teamcode.utils;

public class Numbers {
    /**
     * Converts a -180 to 180 range (like from the IMU) to a 0 to 360 range
     * @param angle The angle, in degrees
     * @return The angle converted to a 0 to 360 range
     */
    public static double normalizeAngle(double angle) {
        return angle > 0 ? 360 - angle : angle;
    }

    /**
     * Rounds a double by x places
     * @param num The double to round
     * @param places The amount of places after the decimal point to round by
     * @return The rounded double
     */
    public static double round(double num, int places) {
        double scalingFactor = Math.pow(10, places);
        return (int) (num * scalingFactor) / scalingFactor;
    }
}
