package org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil;

import com.acmerobotics.roadrunner.Vector2d;

public class MathUtil {

    // Rotates a 2D vector by the specified angle (in radians)
    public static Vector2d rotateVec(Vector2d vec, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        return new Vector2d(
                vec.component1() * cosA - vec.component2() * sinA,
                vec.component1() * sinA + vec.component2() * cosA
        );
    }

    // Applies a deadzone to the input value
    public static double applyDeadzone(double input, double deadzone) {
        return Math.abs(input) < deadzone ? 0 : input;
    }

    // Converts encoder ticks to distance based on counts per revolution (CPR) and diameter
    public static double convertTicksToDistance(double CPR, double diameter, double ticks) {
        return Math.PI * diameter * (ticks / CPR);
    }

    // Converts a distance to encoder ticks based on counts per revolution (CPR) and diameter
    public static double convertDistanceToTicks(double CPR, double diameter, double distance) {
        return (distance / (Math.PI * diameter)) * CPR;
    }

    // Converts encoder ticks to degrees based on counts per revolution (CPR)
    public static double convertTicksToDegrees(double CPR, double ticks) {
        return (ticks / CPR) * 360 % 360;
    }

    // Converts degrees to encoder ticks based on counts per revolution (CPR)
    public static double convertDegreesToTicks(double CPR, double degrees) {
        return (degrees / 360) * CPR;
    }

    // Converts a voltage reading to degrees (assumes 3.3V corresponds to 360 degrees)
    public static double voltageToDegrees(double voltage) {
        return (voltage * 360) / 3.3;
    }

    // Checks if a value is within a specified tolerance of a desired value
    public static boolean inTolerance(double desiredPosition, double currentPosition, double tolerance) {
        return Math.abs(desiredPosition - currentPosition) <= tolerance;
    }

    // Optimizes an angle to minimize rotation (keeps it close to the previous angle)
    public static double optimizeAngle(double angle, double prevAngle) {
        double delta = angle - prevAngle;
        if (Math.abs(delta) > 180.0) {
            angle -= Math.signum(delta) * 360.0;
        }
        return angle;
    }

    // Clamps a value between a minimum and maximum range
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // Linearly interpolates between two values based on a given fraction
    public static double lerp(double start, double end, double fraction) {
        return start + fraction * (end - start);
    }

    // Maps a value from one range to another
    public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return outMin + ((value - inMin) * (outMax - outMin)) / (inMax - inMin);
    }

    // Normalizes an angle to the range [0, 360)
    public static double normalizeAngle(double angle) {
        angle %= 360;
        return angle < 0 ? angle + 360 : angle;
    }

    // Normalizes an angle to the range [-180, 180)
    public static double normalizeAngleTo180(double angle) {
        angle = normalizeAngle(angle);
        return angle > 180 ? angle - 360 : angle;
    }

    // Calculates the distance between two points
    public static double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // Calculates the angle between two points in degrees
    public static double angleBetweenPoints(double x1, double y1, double x2, double y2) {
        return Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));
    }

    // Wraps a value to a specific range [min, max)
    public static double wrap(double value, double min, double max) {
        double range = max - min;
        value = (value - min) % range;
        return value < 0 ? value + max : value + min;
    }
}