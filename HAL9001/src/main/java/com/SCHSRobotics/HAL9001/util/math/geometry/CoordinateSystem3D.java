package com.SCHSRobotics.HAL9001.util.math.geometry;

import static java.lang.Math.acos;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import org.jetbrains.annotations.NotNull;

import java.util.function.Function;

public enum CoordinateSystem3D implements CoordinateSystem<CoordinateSystem3D> {
    CARTESIAN, CYLINDRICAL, SPHERICAL;

    @Override
    public int dimensionality() {
        return 3;
    }

    @Override
    public @NotNull Function<double[], double[]> convertTo(CoordinateSystem3D coordinateSystem) {
        if(this.equals(coordinateSystem)) {
            return (double[] point) -> point;
        }

        switch (coordinateSystem) {
            //convert to cartesian (x, y, z)
            case CARTESIAN:
            default:
                //from cylindrical (r, theta, z)
                if(this.equals(CYLINDRICAL)) {
                    return (double[] point)-> new double[] {point[0]*cos(point[1]), point[0]*sin(point[1]), point[2]};
                }
                //from spherical (rho, phi, theta)
                return (double[] point)-> new double[] {point[0]*cos(point[1])*sin(point[2]), point[0]*sin(point[1])*sin(point[2]), point[0]*cos(point[2])};

            //convert to cylindrical (r, theta, z)
            case CYLINDRICAL:
                //from cartesian (x, y, z)
                if(this.equals(CARTESIAN)) {
                    return (double[] point) -> new double[] {hypot(point[0], point[1]), atan2(point[1], point[0]), point[2]};
                }
                //from spherical (rho, phi, theta)
                return (double[] point) -> new double[] {point[0]*sin(point[1]), point[2], point[0]*cos(point[1])};

            //convert to spherical (rho, phi, theta)
            case SPHERICAL:
                //from cartesian (x, y, z)
                if(this.equals(CARTESIAN)) {
                    return (double[] point) -> {
                        double rho = hypot(hypot(point[0], point[1]), point[2]);
                        return new double[] {rho, acos(point[2]/rho), atan2(point[1], point[0])};
                    };
                }
                //from cylindrical (r, theta, z)
                return (double[] point) -> {
                    double rho = hypot(point[0], point[2]);
                    return new double[] {rho, point[1], acos(point[2]/rho)};
                };
        }
    }
}
