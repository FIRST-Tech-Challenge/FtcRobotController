/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths.curves.bezier;

import org.firstinspires.ftc.teamcode.follower.math.Matrix;
import java.util.HashMap;

/**
 * The BasisMatrixSupplier handles supply the characteristic matrices for splines in their matrix representations.
 *
 * @author William Phomphakdee - 7462 Not to Scale Alumni
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 0.1.0 08/28/2026
 */
public class BasisMatrixSupplier {
    private static final HashMap<Integer, Matrix> bezierMatrices = new HashMap<>();
    private static boolean initialized = false;

    /**
     * This method sets up this class and caches some commonly used Bézier curves,
     * such as the quadratic and cubic Béziers.
     */
    public void initialize() {
        if (!initialized) {
            getBezierCharacteristicMatrix(2); // quadratic bezier
            getBezierCharacteristicMatrix(3); // cubic bezier
            getBezierCharacteristicMatrix(4); // quartic bezier
            getBezierCharacteristicMatrix(5); // quintic bezier
            initialized = true;
        }
    }

    /**
     * This method generate Pascal's triangle with alternating signs (all values are left-aligned in the matrix)
     * @param layers how many layers of the triangle to generate; the minimum should be 1
     * @return Pascal's triangle (left-aligned)
     */
    private static double[][] generatePascalTriangle(int layers) {
        double[][] output = new double[layers][layers];

        // seed the first value of pascal's triangle
        output[0][0] = 1;

        for (int row = 1; row < output.length; row++) {
            // since col is 0 and [row][col - 1] is out of bounds at this moment,
            // the default should be a 0
            double a = 0.0;
            // iterate through the row's elements
            for (int col = 0; col <= row; col++) {
                double b = output[row - 1][col];
                // the ruleset for the typical pascal's triangle is a + b.
                // but, we want alternating signs for cheap without iterating again,
                // so the rule goes to b - a
                output[row][col] = b - a;
                // assign 'a' as 'b' since it will be the correct previous [row - 1] value
                // for the next iteration
                a = b;
            }
        }

        return output;
    }

    /**
     * This method generates the characteristic matrix based on the degree of a requested Bézier curve.
     * 2 for quadratic, 3 for cubic, 4 for quartic, 5 for quintic... etc.
     * @param controlPointCount bezier curve's control point count
     * @return characteristic matrix of the Matrix class
     */
    public static Matrix generateBezierCharacteristicMatrix(int controlPointCount) {
        // get a square matrix that contains Pascal's triangle
        double[][] outputVals = generatePascalTriangle(controlPointCount);

        // sample the last row and multiply all other rows by the corresponding value
        double[] sampledRow = outputVals[outputVals.length - 1];
        for (int i = 1; i < outputVals.length; i++) {
            for (int j = 0; j <= i; j++) {
                outputVals[i][j] = outputVals[i][j] * sampledRow[i];
            }
        }

        return new Matrix(outputVals);
    }

    /**
     * This method gets a characteristic matrix that is stored. If it doesn't exist, generate and return it.
     * @param controlPointCount bezier curve's degree
     * @return characteristic matrix of the Matrix class
     */
    public static Matrix getBezierCharacteristicMatrix(int controlPointCount) {
        return bezierMatrices.computeIfAbsent(
                controlPointCount, BasisMatrixSupplier::generateBezierCharacteristicMatrix);
    }
}
