package org.firstinspires.ftc.team12397.v2.cameraSoftware.util;

public class ExponentialMovingAverage {
    private final double alpha;
    private Double value;

    public ExponentialMovingAverage(double alpha) { this.alpha = alpha; }

    /** @return new filtered value */
    public double update(double sample) {
        value = (value == null) ? sample : alpha * sample + (1 - alpha) * value;
        return value;
    }
}