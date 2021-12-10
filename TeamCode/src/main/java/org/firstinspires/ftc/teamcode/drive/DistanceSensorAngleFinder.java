package org.firstinspires.ftc.teamcode.drive;

public class DistanceSensorAngleFinder {
    public double findAngleBySensors(double distance1, double distance2) {
        double angle = 0;
        double theta2;
        for (theta2 = 0; theta2 < Math.PI * 2; theta2 += 1) {
            double theta1 = theta2 - 90;
            double y1 = Math.sin(theta1) * distance1;
            double y2 = Math.sin(theta2) * distance2;
            if (Math.abs(y1 - y2) < 0.05) {
                angle = theta2 + 90;
            }
        }
        return angle;
    }
}
