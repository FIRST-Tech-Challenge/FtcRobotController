package org.firstinspires.ftc.teamcode.util;

public final class RollingThreeMedian {
    private final double[] history = new double[3];
    private int i;

    public double update(double x) {
        history[i] = x;

        i = (i + 1) % 3;

        return history[0] + history[1] + history[2]
                - Math.min(history[0], Math.min(history[1], history[2]))
                - Math.max(history[0], Math.max(history[1], history[2]));
    }
}
