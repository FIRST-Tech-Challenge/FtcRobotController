package com.wilyworks.simulator.helpers;

public class Globals {
    // Normalize an angle to [180, -180):
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle <= -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }
}
