/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.utils;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class Utils {
    private Utils() {}

    public static Pair<Double, Double> solveQuadratic(double a, double b, double c) {
        double sqrtD = java.lang.Math.sqrt(b * b - 4 * a * c);
        double q = -0.5 * (b + java.lang.Math.copySign(sqrtD, b));
        return Pair.of(q / a, c / q);
    }

    public static double clamp(double num, double lower, double upper) {
        return Math.max(lower, Math.min(num, upper));
    }

    public static double binomial(int n, int k) {
        if (k < 0 || k > n) return 0;
        if (k == 0 || k == n) return 1;
        k = Math.min(k, n - k);
        double result = 1;
        for (int i = 1; i <= k; i++) {
            result = result * (n - (k - i)) / i;
        }
        return result;
    }

    public static double lerp(double a, double b, double t) {
        return (1 - t) * a + t * b;
    }

    public static double[] linspace(double a, double b, int samples) {
        if (samples < 2) {
            throw new IllegalArgumentException("Samples must be >= 2");
        }
        double[] result = new double[samples];
        result[0] = a;
        result[result.length - 1] = b;

        double t;
        for (int i = 1; i < samples - 1; i++) {
            t = i / (samples - 1d);
            result[i] = lerp(a, b, t);
        }

        return result;
    }

    @SafeVarargs
    public static <T> List<T> listOf(T... elements) {
        if (elements.length == 0) return Collections.emptyList();
        return Collections.unmodifiableList(Arrays.asList(elements));
    }

    public static <T> List<T> copyOf(List<T> list) {
        if (list.isEmpty()) return Collections.emptyList();
        else if (list.size() == 1) return Collections.singletonList(list.get(0));
        return list.stream().collect(toUnmodifiableList());
    }

    public static <T> Collector<T, ?, List<T>> toUnmodifiableList() {
        return Collectors.collectingAndThen(Collectors.toList(), Collections::unmodifiableList);
    }

    public static <T> List<T> concat(List<T> lhs, List<T> rhs) {
        if (lhs.isEmpty()) return rhs;
        if (rhs.isEmpty()) return lhs;
        return Stream.concat(lhs.stream(), rhs.stream()).collect(toUnmodifiableList());
    }

    public static double[] linearFit(Double[] x, Double[] y) {
        int n = x.length;
        double sumX = 0, sumXY = 0, sumY = 0, sumX2 = 0;

        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }

        double m = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        double b = (sumY - m * sumX) / n;
        return new double[] {b, m};
    }

    public static double[] quadraticFit(List<double[]> samples) {
        double s11 = 0.0;
        double s12 = 0.0;
        double s22 = 0.0;

        double t1 = 0.0;
        double t2 = 0.0;

        for (double[] sample : samples) {
            double x1 = sample[0];
            double d = sample[1];

            double x2 = x1 * x1;

            s11 += x1 * x1;
            s12 += x1 * x2;
            s22 += x2 * x2;

            t1 += x1 * d;
            t2 += x2 * d;
        }

        double det = s11 * s22 - s12 * s12;
        if (Math.abs(det) < 1e-12) {
            throw new IllegalArgumentException("Regression matrix is singular.");
        }

        double b = (t1 * s22 - t2 * s12) / det;
        double a = (s11 * t2 - s12 * t1) / det;

        return new double[] {b, a};
    }
}
