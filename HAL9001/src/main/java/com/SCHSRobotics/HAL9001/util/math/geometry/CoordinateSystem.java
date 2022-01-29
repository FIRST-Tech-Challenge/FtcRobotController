package com.SCHSRobotics.HAL9001.util.math.geometry;

import java.util.function.Function;

public interface CoordinateSystem<T extends CoordinateSystem<T>> {
    int dimensionality();
    Function<double[], double[]> convertTo(T coordinateSystem);
}
