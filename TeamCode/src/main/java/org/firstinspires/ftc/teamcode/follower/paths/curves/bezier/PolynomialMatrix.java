/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths.curves.bezier;

import org.firstinspires.ftc.teamcode.follower.math.Matrix;
import java.util.Arrays;

public class PolynomialMatrix {
    private final int controlPointCount;

    public PolynomialMatrix(int controlPointCount) {
        this.controlPointCount = controlPointCount;
    }

    public int getControlPointCount() {
        return controlPointCount;
    }

    /**
     * Coefficient of t^(i - diffLevel) in d^diffLevel/dt^diffLevel (t^i):
     * i * (i-1) * ... * (i - diffLevel + 1), i.e. i! / (i - diffLevel)!, or 0 if i < diffLevel.
     * Works for any diffLevel >= 0, not just a fixed hardcoded range.
     */
    private static double fallingFactorial(int i, int diffLevel) {
        if (i < diffLevel) return 0.0;
        double result = 1.0;
        for (int k = 0; k < diffLevel; k++) {
            result *= (i - k);
        }
        return result;
    }

    public Matrix getTMatrix(int diffLevel, double t) {
        double[] output = new double[controlPointCount];
        double tInput = 1.0;
        for (int i = diffLevel; i < controlPointCount; i++) {
            output[i] = tInput * fallingFactorial(i, diffLevel);
            tInput *= t;
        }
        return new Matrix(new double[][] {output});
    }

    public Matrix getTMatrix(int diffLevel, double[] tValues) {
        double[][] output = new double[tValues.length][controlPointCount];
        double[] tInput = new double[tValues.length];
        Arrays.fill(tInput, 1.0);
        for (int i = diffLevel; i < controlPointCount; i++) {
            double coeff = fallingFactorial(i, diffLevel);
            for (int j = 0; j < tInput.length; j++) {
                output[j][i] = tInput[j] * coeff;
                tInput[j] *= tValues[j];
            }
        }
        return new Matrix(output);
    }

    public Matrix getTMatrix(int[] diffLevel, double t) {
        double[][] output = new double[diffLevel.length][controlPointCount];
        double[] tInput = new double[controlPointCount];
        tInput[0] = 1;
        for (int i = 1; i < tInput.length; i++) {
            tInput[i] = tInput[i - 1] * t;
        }
        for (int diffIdx = 0; diffIdx < diffLevel.length; diffIdx++) {
            int d = diffLevel[diffIdx];
            for (int i = d; i < controlPointCount; i++) {
                output[diffIdx][i] = fallingFactorial(i, d) * tInput[i - d];
            }
        }
        return new Matrix(output);
    }
}
