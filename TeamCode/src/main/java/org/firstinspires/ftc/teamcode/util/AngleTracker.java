package org.firstinspires.ftc.teamcode.util;

/**
 * Tracks continuous angle rotation without wraparound issues
 */
public class AngleTracker {
    private double totalRotation;
    private double previousAngle;
    private boolean firstReading;

    public AngleTracker() {
        totalRotation = 0.0;
        previousAngle = 0.0;
        firstReading = true;
    }

    /**
     * Update the total rotation based on new angle reading
     *
     * @param currentAngle Current angle in radians (-π to π)
     * @return Total accumulated rotation in radians
     */
    public double update(double currentAngle) {
        if (firstReading) {
            previousAngle = currentAngle;
            firstReading = false;
            return 0.0;
        }

        // Get shortest difference
        double diff = AngleUtil.normalizeAngleDifference(currentAngle, previousAngle);

        // Update total and previous
        totalRotation += diff;
        previousAngle = currentAngle;

        return totalRotation;
    }

    /**
     * Get the total accumulated rotation
     *
     * @return Total rotation in radians
     */
    public double getTotalRotation() {
        return totalRotation;
    }

    /**
     * Reset the tracker to initial state
     */
    public void reset() {
        totalRotation = 0.0;
        previousAngle = 0.0;
        firstReading = true;
    }
}