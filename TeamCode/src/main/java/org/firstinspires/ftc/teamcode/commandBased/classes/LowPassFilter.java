package org.firstinspires.ftc.teamcode.commandBased.classes;

public class LowPassFilter {

    private double gain;

    private double previousEstimate;

    public LowPassFilter(double gain) {
        this.gain = gain;
    }

    public double estimate(double measure) {
        double estimate = gain * previousEstimate + (1 - gain) * measure;
        previousEstimate = estimate;
        return estimate;
    }
}
