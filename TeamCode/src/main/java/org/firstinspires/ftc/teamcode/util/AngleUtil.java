package org.firstinspires.ftc.teamcode.util;

public class AngleUtil {
    /**
     * Calculate the shortest angular difference between two angles in radians,
     * handling the wraparound case correctly.
     *
     * @param current Current angle in radians (-π to π)
     * @param previous Previous angle in radians (-π to π)
     * @return Shortest angular difference in radians
     */
    public static double normalizeAngleDifference(double current, double previous) {
        double diff = current - previous;

        // Normalize to -π to π
        if (diff > Math.PI) {
            diff -= 2 * Math.PI;
        } else if (diff < -Math.PI) {
            diff += 2 * Math.PI;
        }

        return diff;
    }
}
