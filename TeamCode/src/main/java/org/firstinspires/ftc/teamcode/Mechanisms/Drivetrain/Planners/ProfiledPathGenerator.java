package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Planners;

import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;


class ProfiledPathGenerator {
    public static double[][] generatePath(SimpleMatrix rawWaypoints) {
        // Get little n
        int n = rawWaypoints.getNumRows() - 1;

        // Build A, b matrices
        SimpleMatrix[] continuityConstraints = constructContinuityConstraints(n, rawWaypoints);
        SimpleMatrix aCont = continuityConstraints[0];
        SimpleMatrix bCont = continuityConstraints[1];
        SimpleMatrix[] SmoothnessConstraints = constructSmoothnessConstraints(n, rawWaypoints);
        SimpleMatrix aSmoo = SmoothnessConstraints[0];
        SimpleMatrix bSmoo = SmoothnessConstraints[1];
        SimpleMatrix[] additionalConstraints = constructAdditionalConstrains(n, rawWaypoints);
        SimpleMatrix aAdd = additionalConstraints[0];
        SimpleMatrix bAdd = additionalConstraints[1];
        // Stack the matrices
        SimpleMatrix A = new SimpleMatrix(8 * n, 8 * n);
        A.insertIntoThis(0, 0, aCont);
        A.insertIntoThis(4 * n, 0, aSmoo);
        A.insertIntoThis((4 * n - 4) + 4 * n, 0, aAdd);
        SimpleMatrix b = new SimpleMatrix(8 * n, 1);
        b.insertIntoThis(0, 0, bCont);
        b.insertIntoThis(4 * n, 0, bSmoo);
        b.insertIntoThis((4 * n - 4) + 4 * n, 0, bAdd);
        // Solve the system of equations
        SimpleMatrix x = A.invert().mult(b);
        System.out.println(A.getNumRows());
        System.out.println(A.getNumCols());
        System.out.println(b.getNumRows());
        System.out.println(b.getNumCols());
        // solve with large A and b
        // discretize the path
        // return the discretized path
        return discretizePath(x, 50, n);
    }

    /**
     * Generates the constraints to connect the different polynomials.
     * @param n (int) Number of polynomials to create
     * @param rawWaypoints (SimpleMatrix) Waypoints entered by the user
     * @return (SimpleMatrix[]) The A and b matrices associated with these constraints
     */
    private static SimpleMatrix[] constructContinuityConstraints(int n, SimpleMatrix rawWaypoints) {
        // Build the matrices ACont, bCont,
        SimpleMatrix aiCont = new SimpleMatrix(
                new double[][]{
                        {0, 0, 0, 1, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 1},
                        {1, 1, 1, 1, 0, 0, 0, 0},
                        {0, 0, 0, 0, 1, 1, 1, 1}
                }
        );
        SimpleMatrix bCont = new SimpleMatrix(n * 4, 1);
        for (int i = 1; i < n + 1; i++) {
            bCont.set((4 * i - 4), 0, rawWaypoints.get(i - 1, 0));
            bCont.set(4 * i - 3, 0, rawWaypoints.get(i - 1, 1));
            bCont.set((4 * i - 2), 0, rawWaypoints.get(i, 0));
            bCont.set((4 * i - 1), 0, rawWaypoints.get(i, 1));
        }
        SimpleMatrix I = SimpleMatrix.identity(n);
        SimpleMatrix aCont = I.kron(aiCont);
        return new SimpleMatrix[]{aCont, bCont};
    }

    private static SimpleMatrix[] constructSmoothnessConstraints(int n, SimpleMatrix rawWaypoints) {
        SimpleMatrix AjSmoo = new SimpleMatrix(
                new double[][]{
                        {3, 2, 1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, -1, 0},
                        {3, 1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 3, 1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0}
                }
        );
        SimpleMatrix ASmoo = new SimpleMatrix(4 * (n - 1), 8 * n);
        ASmoo.zero();
        for (int i = 0; i < n - 1; i++) {
            ASmoo.insertIntoThis(4 * i, 8 * i, AjSmoo);
        }
        SimpleMatrix bSmoo = new SimpleMatrix(4 * (n - 1), 1);
        bSmoo.zero();
        // array of size 2
        SimpleMatrix[] constraints = new SimpleMatrix[2];
        constraints[0] = ASmoo;
        constraints[1] = bSmoo;

        return constraints;
    }

    /***
     * Generates the additional constraints needed for multiplication.
     * @param n (int) Number of polynomials to create
     * @param rawWaypoints (SimpleMatrix) Waypoints entered by the user
     * @return (SimpleMatrix[]) The A and b matrices associated with these constraints
     */
    private static SimpleMatrix[] constructAdditionalConstrains(int n, SimpleMatrix rawWaypoints) {
        SimpleMatrix aAdd = new SimpleMatrix(4, 8 * n);
        aAdd.zero();
        aAdd.set(0, 2, 1);
        aAdd.set(1, 6, 1);
        aAdd.set(2, 8 * n - 6, 1);
        aAdd.set(2, 8 * n - 7, 2);
        aAdd.set(2, 8 * n - 8, 3);
        aAdd.set(3, 8 * n - 2, 1);
        aAdd.set(3, 8 * n - 3, 2);
        aAdd.set(3, 8 * n - 4, 3);
        SimpleMatrix bAdd = new SimpleMatrix(4, 1);
        bAdd.zero();
        SimpleMatrix[] constraints = new SimpleMatrix[2];
        constraints[0] = aAdd;
        constraints[1] = bAdd;
        return constraints;
    }

    private static double[][] discretizePath(SimpleMatrix polynomialConstants, int N, int n) {
        double[] T = new double[N + 1];
        for (int i = 0; i <= N; i++) {
            T[i] = (double) i / N;
        }

        SimpleMatrix points = new SimpleMatrix(2, (N * n) + 1);

        double[][] equations = new double[polynomialConstants.getNumRows() / 8][8];
        for (int i = 0; i < polynomialConstants.getNumRows(); i += 8) {
            for (int j = 0; j < 8; j++) {
                equations[i / 8][j] = polynomialConstants.get(i + j);
            }
        }

        for (int i = 0; i < equations.length; i++) {
            for (int j = 0; j <= N; j++) {
                double sumX = equations[i][0] * Math.pow(T[j], 3) + equations[i][1] * Math.pow(T[j], 2) + equations[i][2] * T[j] + equations[i][3];
                points.set(0, N * i + j, sumX);

                double sumY = equations[i][4] * Math.pow(T[j], 3) + equations[i][5] * Math.pow(T[j], 2) + equations[i][6] * T[j] + equations[i][7];
                points.set(1, N * i + j, sumY);
            }
        }

        points.set(0, N * n, equations[n - 1][0] + equations[n - 1][1] + equations[n - 1][2] + equations[n - 1][3]);
        points.set(1, N * n, equations[n - 1][4] + equations[n - 1][5] + equations[n - 1][6] + equations[n - 1][7]);

        return points.toArray2();
    }
}
