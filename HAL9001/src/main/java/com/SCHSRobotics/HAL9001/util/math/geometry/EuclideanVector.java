package com.SCHSRobotics.HAL9001.util.math.geometry;

import static java.lang.Math.acos;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

public abstract class EuclideanVector<V extends EuclideanVector<V>> implements Vector<V>{
    protected final MatrixSimple vectorMatrix;

    public EuclideanVector(@NotNull MatrixSimple vectorMatrix) {
        ExceptionChecker.assertTrue(vectorMatrix.isVector(), new ArithmeticException("Matrix is not a vector."));
        this.vectorMatrix = vectorMatrix;
    }

    public EuclideanVector(double... components) {
        this(new MatrixSimple(components).transpose());
    }

    @Override
    public V set(@NotNull V vector) {
        return set(vector.vectorMatrix);
    }

    @Override
    public V set(@NotNull MatrixSimple vectorMatrix) {
        ExceptionChecker.assertTrue(vectorMatrix.isVector(), new ArithmeticException("matrix is not a vector."));
        ExceptionChecker.assertEqual(dim(), vectorMatrix.getNumRows(), new ArithmeticException("dimensionality of vectors does not match."));

        for (int row = 0; row < vectorMatrix.getNumRows(); row++) {
            this.vectorMatrix.set(row, 0, vectorMatrix.get(row, 0));
        }
        return (V) this;
    }

    @Override
    public final MatrixSimple toMatrix() {
        return vectorMatrix;
    }

    @Override
    public final double @NotNull [] getComponents() {
        double[] components = new double[vectorMatrix.getNumRows()];
        for (int row = 0; row < vectorMatrix.getNumRows(); row++) {
            components[row] = vectorMatrix.get(row, 0);
        }
        return components;
    }

    public final boolean isZeroVector() {
        return vectorMatrix.isZeroMatrix();
    }

    public final double dot(@NotNull V vector) {
        return vectorMatrix.transpose().multiply(vector.vectorMatrix).get(0, 0);
    }

    @Override
    public V scaleTo(double magnitude) {
        return copy().normalize().multiply(magnitude);
    }

    @Override
    public final boolean isNormalTo(V vector) {
        return this.dot(vector) == 0;
    }

    public final double magnitudeSquared() {
        return this.dot((V) this);
    }

    @Override
    public final double magnitude() {
        return sqrt(magnitudeSquared());
    }

    @Override
    public final boolean isUnitVector() {
        return this.magnitude() == 1;
    }

    public final double angleTo(V vector, HALAngleUnit angleUnit) {
        double magnitude = this.magnitude();
        return HALAngleUnit.RADIANS.convertTo(angleUnit).apply(acos(this.dot(vector)/(magnitude*magnitude)));
    }

    @Override
    public final double angleTo(V vector) {
        return angleTo(vector, HALAngleUnit.RADIANS);
    }

    @Override
    public int dim() {
        return vectorMatrix.getNumRows();
    }

    public V normalize() {
        ExceptionChecker.assertFalse(this.isZeroVector(), new ArithmeticException("Cannot normalize the zero vector."));
        double magnitude = this.magnitude();
        return copy().set(vectorMatrix.divide(magnitude));
    }

    public V multiply(double scalar) {
        return copy().set(vectorMatrix.multiply(scalar));
    }

    public V divide(double scalar) {
        ExceptionChecker.assertNotEqual(scalar, 0, new ArithmeticException("cannot divide by 0"));
        return multiply(1/scalar);
    }

    public V negate() {
        return multiply(-1);
    }

    public V add(@NotNull V vector) {
        return copy().set(vectorMatrix.add(vector.vectorMatrix));
    }

    public V subtract(@NotNull V vector) {
        return copy().set(vectorMatrix.subtract(vector.vectorMatrix));
    }

    public V project(@NotNull V ontoVector) {
        return ontoVector.multiply(this.dot(ontoVector)/ontoVector.dot(ontoVector));
    }

    public V rotate(@NotNull Axis<V> u, @NotNull Axis<V> v, double angle, @NotNull HALAngleUnit angleUnit) {
        ExceptionChecker.assertTrue(u.getAxisVector().isNormalTo(v.getAxisVector()), new RuntimeException("Vectors defining the rotation object must be orthonormal."));

        MatrixSimple uMatrix = u.getAxisVector().toMatrix();
        MatrixSimple vMatrix = v.getAxisVector().toMatrix();

        MatrixSimple a = vMatrix.multiply(uMatrix.transpose()).add(uMatrix.multiply(vMatrix.transpose()));
        MatrixSimple b = uMatrix.multiply(uMatrix.transpose()).subtract(vMatrix.multiply(vMatrix.transpose()));

        double theta = angleUnit.convertTo(HALAngleUnit.RADIANS).apply(angle);

        MatrixSimple rotationMatrix = MatrixSimple.identityMatrix(vectorMatrix.getNumRows())
                .add(
                        a.multiply(sin(theta))
                )
                .add(
                        b.multiply(cos(theta)-1)
                );
        return copy().set(rotationMatrix.multiply(vectorMatrix));
    }

    public final V rotate(Axis<V> u, Axis<V> v, double angle) {
        return rotate(u, v, angle, HALAngleUnit.RADIANS);
    }

    @Override
    public String toString() {
        String componentArrayString = Arrays.toString(getComponents());
        return "<" + componentArrayString.substring(1,componentArrayString.length()-1) + '>';
    }
}
