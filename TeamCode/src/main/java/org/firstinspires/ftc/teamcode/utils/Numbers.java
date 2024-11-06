package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;

import java.util.function.BiFunction;
import java.util.function.Consumer;

public class Numbers {
    /**
     * Converts a -180 to 180 range (like from the IMU) to a 0 to 360 range
     * @param angle The angle, in degrees
     * @return The angle converted to a 0 to 360 range
     */
    public static double normalizeAngle(double angle) {
        return Math.abs(angle > 0 ? 360 - angle : angle);
    }

    public static double clip(double number, double min, double max) {
        if (number < min) {
            return min;
        } else {
            return Math.min(number, max);
        }
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

    /**
     * Given the target rotation of the robot and the current rotation of the robot, both in degrees, return the appropriate turn correction direction and magnitude [-1, 1]
     * @param target The current rotation of the robot
     * @return The turn correction speed
     */
    private static final double _tolerance = 0.5;
    public static double turnCorrectionSpeed(double current, double target) {
        double left = normalizeAngle(target - current);
        double right =normalizeAngle(current - target);
        double angle = Math.abs(left) < Math.abs(right) ? -left : right;
        if (Math.abs(angle) < _tolerance) return 0;
        return Range.clip(angle / 45, -1, 1);
    }

    /**
     * Linearly interpolate between values a and b according to t
     * @param a The starting value, what will be returned when t = 0
     * @param b The ending value, what will be returned when t = 1
     * @param t The interpolation value
     * @return A number between a and b, chosen linearly by t
     */
    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    /**
     * Quadratically interpolate between values a and b according t
     * @param a The starting value, what will be returned when t = 0
     * @param b The ending value, what will be returned when t = 1
     * @param t The interpolation value
     * @return A number between a and b, chosen quadratically by t
     */
    public static double querp(double a, double b, double t) {
        return lerp(a, b, Math.abs(t) * t);
    }
}
