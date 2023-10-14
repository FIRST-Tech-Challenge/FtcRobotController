package org.firstinspires.ftc.teamcode.lib.util;

/**
 * A Long that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 */
public class InterpolatingLong implements Interpolable<InterpolatingLong>, InverseInterpolable<InterpolatingLong>,
        Comparable<InterpolatingLong> {
    public Long value = 0L;

    public InterpolatingLong(Long val) {
        value = val;
    }

    @Override
    public InterpolatingLong interpolate(InterpolatingLong other, double x) {
        Long dydx = other.value - value;
        Double searchY = dydx * x + value;
        return new InterpolatingLong(searchY.longValue());
    }

    @Override
    public double inverseInterpolate(InterpolatingLong upper, InterpolatingLong query) {
        long upper_to_lower = upper.value - value;
        if (upper_to_lower <= 0) {
            return 0;
        }
        long query_to_lower = query.value - value;
        if (query_to_lower <= 0) {
            return 0;
        }
        return query_to_lower / (double) upper_to_lower;
    }

    @Override
    public int compareTo(InterpolatingLong other) {
        if (other.value < value) {
            return 1;
        } else if (other.value > value) {
            return -1;
        } else {
            return 0;
        }
    }
}