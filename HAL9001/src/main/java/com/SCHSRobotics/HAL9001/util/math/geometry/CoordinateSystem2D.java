package com.SCHSRobotics.HAL9001.util.math.geometry;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import org.jetbrains.annotations.NotNull;

import java.util.function.Function;

public enum CoordinateSystem2D implements CoordinateSystem<CoordinateSystem2D> {
    CARTESIAN, POLAR;

    @Override
    public int dimensionality() {
        return 2;
    }

    @Override
    public @NotNull Function<double[], double[]> convertTo(CoordinateSystem2D coordinateSystem) {
        if(this.equals(coordinateSystem)) {
            return (double[] point) -> point;
        }
        else if(this.equals(CARTESIAN)) {
            return (double[] point) -> {
                double x = point[0];
                double y = point[1];
                return new double[] {hypot(x, y), atan2(y, x)};
            };
        }
        return (double[] point) -> {
            double r = point[0];
            double theta = point[1];
            return new double[] {r*cos(theta), r*sin(theta)};
        };
    }
}
