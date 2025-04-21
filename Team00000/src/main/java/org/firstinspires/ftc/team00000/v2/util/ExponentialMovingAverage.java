package org.firstinspires.ftc.team00000.v2.util;

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