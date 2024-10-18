package org.firstinspires.ftc.teamcode.util;

public class Numbers {
    /**
     * Converts a -180 to 180 range (like from the IMU) to a 0 to 360 range
     * @param angle The angle, in degrees
     * @return The angle converted to a 0 to 360 range
     */
    public static double normalizeAngle(double angle) {
        return Math.abs(angle > 0 ? 360 - angle : angle);
    }

    /**
     * Rounds a double by x places
     * @param num The double to round
     * @param places The amount of places after the decimal point to round by
     * @return The rounded double
     */
    public static double round(double num, int places) {
        String roundFormat = "%." + places + "f";
        return Double.parseDouble(String.format(roundFormat, num));
    }

    /**
     * Applies a "deadzone" to a double, where if that double is within the deadzone it becomes 0
     * @param num The double to apply the deadzone to
     * @param deadzone The smallest value (the absolute value of) num can be before it is in the deadzone
     * @return 0 if the absolute value of num is <= the deadzone, otherwise returns num
     */
    public static double deadzone(double num, double deadzone) {
        return Math.abs(num) > deadzone ? num : 0;
    }

    /**
     * Applies a "deadzone" to a float, where if that float is within the deadzone it becomes 0
     * @param num The float to apply the deadzone to
     * @param deadzone The smallest value (the absolute value of) num can be before it is in the deadzone
     * @return 0 if the absolute value of num is <= the deadzone, otherwise returns num
     */
    public static float deadzone(float num, float deadzone) {
        return Math.abs(num) > deadzone ? num : 0;
    }

    /**
     * Normalizes a double from a specific range into a new range
     * Given x and the range of numbers it could be [min, max]
     * Return a new double scaled to the range [newMin, newMax]
     * @param x The number to scale
     * @param min The smalled double x could be
     * @param max The largest double x could be
     * @param newMin The new minimum that x should be limited to
     * @param newMax The new maximum that x should be limited to
     * @return X scaled from [min, max] to [newMin, newMax]
     */
    public static double normalizeInRange(double x, double min, double max, double newMin, double newMax) {
        return (newMax - newMin) * ((x - min) / (max - min)) + newMin;
    }
}
