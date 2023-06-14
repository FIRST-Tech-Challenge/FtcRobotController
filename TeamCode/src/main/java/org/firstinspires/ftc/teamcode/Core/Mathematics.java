package org.firstinspires.ftc.teamcode.Core;

public class Mathematics {
    public static double[] rotate(double[] vector, double angle) {
        final double[] newVector = {0, 0};
        newVector[0] = Math.cos(angle) * vector[0] + (-Math.sin(angle)) * vector[1];
        newVector[1] = Math.sin(angle) * vector[0] + Math.cos(angle) * vector[1];
        return newVector;
    }
}
