package org.firstinspires.ftc.teamcode.robots.csbot.util;

/**
 * written by ACME Robotics (https://github.com/acmerobotics/robomatic/blob/master/src/main/java/com/acmerobotics/robomatic/util/ExponentialSmoother.java)
 */

public class ExponentialSmoother {

    private double smoothingFactor, average;
    private boolean hasUpdated;

    public ExponentialSmoother(double smoothingFactor) {
        this.smoothingFactor = smoothingFactor;
    }

    /**
     * Update the smoother with a new data value
     * @param newValue new data value
     * @return the smoothed value
     */
    public double update(double newValue) {
        if (!hasUpdated) {
            average = newValue;
            hasUpdated = true;
        } else {
            average = smoothingFactor * newValue + (1 - smoothingFactor) * average;
        }
        return average;
    }

    public void reset() {
        hasUpdated = false;
    }

    public void setSmoothingFactor(double smoothingFactor) {
        this.smoothingFactor = smoothingFactor;
    }
}