package com.SCHSRobotics.HAL9001.util.math.geometry;

import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;

import org.jetbrains.annotations.NotNull;

public class Point3D extends EuclideanPoint<Point3D, Vector3D> {
    private static final Point3D ORIGIN = new Point3D(0,0,0);

    public Point3D(double a, double b, double c, @NotNull CoordinateSystem3D coordinateSystem) {
        super(new Vector3D(
                new MatrixSimple(coordinateSystem.convertTo(CoordinateSystem3D.CARTESIAN).apply(
                        new double[] {a, b, c}
                )).transpose()
        ));
    }

    public Point3D(double x, double y, double z) {
        this(x, y, z, CoordinateSystem3D.CARTESIAN);
    }

    public Point3D(double r, double theta, @NotNull HALAngleUnit angleUnit, double z) {
        this(
                r,
                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(theta),
                z,
                CoordinateSystem3D.CYLINDRICAL
        );
    }
    public Point3D(double r, double inclination, @NotNull HALAngleUnit inclinationUnit, double azimuth, @NotNull HALAngleUnit azimuthUnit) {
        this(
                r,
                inclinationUnit.convertTo(HALAngleUnit.RADIANS).apply(inclination),
                azimuthUnit.convertTo(HALAngleUnit.RADIANS).apply(azimuth),
                CoordinateSystem3D.SPHERICAL
        );
    }

    public Point3D(Vector3D vector) {
        super(vector);
    }

    @Override
    public Point3D copy() {
        return new Point3D(pointVector);
    }

    public double getX() {
        return pointVector.getX();
    }

    public double getY() {
        return pointVector.getY();
    }

    public double getZ() {
        return pointVector.getZ();
    }

    public double getR() {
        return pointVector.magnitude();
    }
    public double getInclination() {
        return pointVector.getInclination();
    }
    public double getInclination(HALAngleUnit angleUnit) {
        return pointVector.getInclination(angleUnit);
    }

    public double getAzimuth() {
        return pointVector.getAzimuth();
    }
    public double getAzimuth(HALAngleUnit angleUnit) {
        return pointVector.getAzimuth(angleUnit);
    }

    public void setX(double x) {
        pointVector.setX(x);
    }
    public void setY(double y) {
        pointVector.setY(y);
    }
    public void setR(double r) {
        pointVector.scaleTo(r);
    }

    public void setInclination(double thetaRadians) {
        pointVector.setInclination(thetaRadians);
    }
    public void setInclination(double theta, HALAngleUnit angleUnit) {
        pointVector.setInclination(theta, angleUnit);
    }

    public void setAzimuth(double thetaRadians) {
        pointVector.setAzimuth(thetaRadians);
    }
    public void setAzimuth(double theta, HALAngleUnit angleUnit) {
        pointVector.setAzimuth(theta, angleUnit);
    }

    public static Point3D getOrigin() {
        return ORIGIN.copy();
    }

    @Override
    public String toString() {
        return "(" + getX() + ", " + getY() + ")";
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Point3D point2D = (Point3D) o;
        return Double.compare(point2D.getX(), getX()) == 0 &&
                Double.compare(point2D.getY(), getY()) == 0 &&
                Double.compare(point2D.getZ(), getZ()) == 0;
    }
}
