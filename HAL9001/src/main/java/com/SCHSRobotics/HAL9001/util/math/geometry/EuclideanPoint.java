package com.SCHSRobotics.HAL9001.util.math.geometry;

import static java.lang.Math.sqrt;

import org.jetbrains.annotations.NotNull;

public abstract class EuclideanPoint<P extends EuclideanPoint<P,V>, V extends EuclideanVector<V>> implements Point<P,V> {

    protected final V pointVector;
    public EuclideanPoint(V pointVector) {
        this.pointVector = pointVector;
    }

    @Override
    public double[] getCoordinates() {
        return pointVector.getComponents();
    }

    @Override
    public double distanceTo(@NotNull P point) {
        double[] coordinates1 = getCoordinates();
        double[] coordinates2 = point.getCoordinates();
        double dst = 0;
        for (int i = 0; i < coordinates1.length; i++) {
            double diff = coordinates1[i]-coordinates2[i];
            dst += diff * diff;
        }
        return sqrt(dst);
    }

    @Override
    public double distanceTo(@NotNull Line<P,V> line) {
        return line.distanceTo((P) this);
    }

    @Override
    public V vectorTo(@NotNull P point) {
        return pointVector.copy().set(point.pointVector.subtract(pointVector));
    }

    public int dim() {
        return pointVector.dim();
    }

}
