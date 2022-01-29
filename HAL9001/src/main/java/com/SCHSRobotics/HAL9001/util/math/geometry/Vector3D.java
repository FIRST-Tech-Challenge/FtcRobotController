package com.SCHSRobotics.HAL9001.util.math.geometry;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;

import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

public class Vector3D extends EuclideanVector<Vector3D> {

    private static final Vector3D ZERO_VECTOR = new Vector3D(0, 0, 0);

    public Vector3D(MatrixSimple vectorMatrix) {
        super(vectorMatrix);
        ExceptionChecker.assertTrue(vectorMatrix.getNumRows() == 3, new ArithmeticException("Matrix vector is wrong size."));
    }

    public Vector3D(@NotNull Point3D start, Point3D end) {
        super(start.vectorTo(end).vectorMatrix);
    }
    public Vector3D(Point3D end) {
        this(Point3D.getOrigin(), end);
    }

    public Vector3D(double x, double y, double z) {
        super(x, y, z);
    }

    public Vector3D(double a, double b, double c, @NotNull CoordinateSystem3D coordinateSystem) {
        super(coordinateSystem.convertTo(CoordinateSystem3D.CARTESIAN).apply(new double[] {a, b, c}));
    }
    public Vector3D(double r, double azimuth, @NotNull HALAngleUnit angleUnit, double z) {
        this(r, angleUnit.convertTo(HALAngleUnit.RADIANS).apply(azimuth), z, CoordinateSystem3D.CYLINDRICAL);
    }

    public Vector3D(double r, double inclination, @NotNull HALAngleUnit inclinationUnit, double azimuth, @NotNull HALAngleUnit azimuthUnit) {
        this(
                r,
                inclinationUnit.convertTo(HALAngleUnit.RADIANS).apply(inclination),
                azimuthUnit.convertTo(HALAngleUnit.RADIANS).apply(azimuth),
                CoordinateSystem3D.SPHERICAL
        );
    }

    @Contract("_ -> new")
    public static @NotNull Vector3D fromND(@NotNull VectorND vector) {
        ExceptionChecker.assertEqual(vector.dim(), 3, new ArithmeticException("Vector dimensionality does not match."));
        return new Vector3D(vector.vectorMatrix.copy());
    }

    @Override
    public Vector3D copy() {
        return new Vector3D(vectorMatrix.copy());
    }

    public double getX() {
        return vectorMatrix.get(0,0);
    }
    public double getY() {
        return vectorMatrix.get(1,0);
    }
    public double getZ() {
        return vectorMatrix.get(2,0);
    }
    public double getAxialDistance() {
        return sqrt(getX()*getX() + getY()*getY());
    }
    public double getAzimuth() {
        double x = getX();
        double y = getY();

        if(x == 0 && y == 0) return 0;
        else return atan2(y, x);
    }
    public double getAzimuth(HALAngleUnit angleUnit) {
        return HALAngleUnit.RADIANS.convertTo(angleUnit).apply(getAzimuth());
    }
    public double getInclination() {
        return atan2(getAxialDistance(), getZ());
    }
    public double getInclination(HALAngleUnit angleUnit) {
        return HALAngleUnit.RADIANS.convertTo(angleUnit).apply(getInclination());
    }

    public void setX(double x) {
        vectorMatrix.set(0,0, x);
    }
    public void setY(double y) {
        vectorMatrix.set(1,0, y);
    }
    public void setZ(double z) {
        vectorMatrix.set(2,0, z);
    }
    public void setAzimuth(double azimuthRadians) {
        setX(getAxialDistance()*cos(azimuthRadians));
        setY(getAxialDistance()*sin(azimuthRadians));
    }
    public void setAzimuth(double azimuth, @NotNull HALAngleUnit angleUnit) {
        setAzimuth(angleUnit.convertTo(HALAngleUnit.RADIANS).apply(azimuth));
    }
    public void setInclination(double inclinationRadians) {
        double mag = magnitude();
        double azimuth = getAzimuth();
        setX(mag*cos(azimuth)*sin(inclinationRadians));
        setY(mag*sin(azimuth)*sin(inclinationRadians));
        setZ(mag*cos(inclinationRadians));
    }
    public void setInclination(double inclination, @NotNull HALAngleUnit angleUnit) {
        setInclination(angleUnit.convertTo(HALAngleUnit.RADIANS).apply(inclination));
    }

    public Vector3D rotate(double angle, @NotNull HALAngleUnit angleUnit, @NotNull Axis3D rotationAxis) {
        double theta = angleUnit.convertTo(HALAngleUnit.RADIANS).apply(angle);
        double cosine = cos(theta);
        double sine = sin(theta);
        double oneMinusCosine = 1 - cosine;
        double x = rotationAxis.getX(), y = rotationAxis.getY(), z = rotationAxis.getZ();
        MatrixSimple rotationMatrix = new MatrixSimple(new double[][] {
                {cosine + x*x*oneMinusCosine,  x*y*oneMinusCosine - z*sine, y*sine + x*z*oneMinusCosine},
                {z*sine + x*y*oneMinusCosine,  cosine + y*y*oneMinusCosine, -x*sine + y*z*oneMinusCosine},
                {-y*sine + x*z*oneMinusCosine, x*sine + y*z*oneMinusCosine, cosine + z*z*oneMinusCosine}
        });

        return copy().set(rotationMatrix.multiply(vectorMatrix));
    }

    public Vector3D cross(@NotNull Vector3D vector) {
        return new Vector3D(
                this.getY()*vector.getZ() - this.getZ()*vector.getY(),
                this.getZ()*vector.getX() - this.getX()*vector.getZ(),
                this.getX()*vector.getY() - this.getY()*vector.getX()
        );
    }

    public static Vector3D zeroVector() {
        return ZERO_VECTOR.copy();
    }
}