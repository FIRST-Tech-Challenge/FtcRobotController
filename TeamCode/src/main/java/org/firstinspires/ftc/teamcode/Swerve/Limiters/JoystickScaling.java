package org.firstinspires.ftc.teamcode.Swerve.Limiters;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;

public class JoystickScaling {
    private double input, intercept, splinePoint, slope;

    public double Scale(double input, double intercept, double splinePoint, double slope) {
        return Math.signum(input) * intercept
                + (1 - intercept)
                * (Math.abs(input) > splinePoint
                    ? Math.pow(Math.abs(input),
                    Math.log(splinePoint / slope)
                        / Math.log(splinePoint))
                * Math.signum(input) : input / slope);
    }

    public Point ScaleVector(Point Other) {
        double magnitude = hypot(Other.x, Other.y);

        double scaledMagnitude = Scale(magnitude, 0.001, 0.66, 4);

        double direction = atan2(Other.y, Other.x);
        return new Point(
                Math.cos(direction) * scaledMagnitude,
                Math.sin(direction) * scaledMagnitude
        );
    }
}
