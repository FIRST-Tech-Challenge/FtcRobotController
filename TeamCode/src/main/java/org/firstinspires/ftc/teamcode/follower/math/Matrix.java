/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.math;

import org.firstinspires.ftc.teamcode.follower.utils.Pair;
import java.util.Arrays;
import java.util.Locale;

/**
 * Represents a mathematical matrix of doubles.
 * Provides basic linear algebra operations including addition,
 * multiplication, and transposition.
 */
public class Matrix {
    public final int rows;
    public final int cols;
    public final double[] data;

    /**
     * Constructs a new matrix from an existing 2D array.
     * Performs a deep copy to ensure the internal state is encapsulated.
     *
     * @param data A 2D array of doubles.
     */
    public Matrix(double[][] data) {
        this.rows = data.length;
        this.cols = data[0].length;
        this.data = new double[rows * cols];
        for (int i = 0; i < rows; i++) {
            if (data[i].length != cols)
                throw new IllegalArgumentException(String.format(
                        "Matrix row 0 had length %d but matrix row %d had length %d", cols, i, data[i].length));
            System.arraycopy(data[i], 0, this.data, i * cols, cols);
        }
    }

    public Matrix(double[] data, int rows, int cols) {
        if (data == null) {
            throw new IllegalArgumentException("Data array cannot be null");
        }
        if (rows <= 0 || cols <= 0) {
            throw new IllegalArgumentException("Rows and columns must be positive integers");
        }
        if (data.length != rows * cols) {
            throw new IllegalArgumentException(String.format(
                    "Array length %d does not match specified dimensions %dx%d (expected %d elements)",
                    data.length, rows, cols, rows * cols));
        }

        this.rows = rows;
        this.cols = cols;
        this.data = new double[rows * cols];
        System.arraycopy(data, 0, this.data, 0, data.length);
    }

    public static Matrix diag(double... eigenvalues) {
        double[][] data = new double[eigenvalues.length][eigenvalues.length];
        for (int i = 0; i < eigenvalues.length; i++) data[i][i] = eigenvalues[i];
        return new Matrix(data);
    }

    public static Matrix diag(Vector vector) {
        return diag(vector.elements);
    }

    public static Matrix diag(Vector2D vector) {
        return diag(vector.x(), vector.y());
    }

    public static Matrix identity(int n) {
        double[][] data = new double[n][n];
        for (int i = 0; i < n; i++) data[i][i] = 1;
        return new Matrix(data);
    }

    public static Matrix zero(int n) {
        double[][] data = new double[n][n];
        return new Matrix(data);
    }

    public static Matrix rotation(double theta) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        return new Matrix(new double[][] {
            {cos, -sin},
            {sin, cos}
        });
    }

    /**
     * Create a 3x3 matrix with a 2d rotation minor matrix on the top left
     * @param theta radians; + = CCW, - = CW
     * @return 3x3 affine rotation matrix
     */
    public static Matrix rotationTransform(double theta) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        return new Matrix(new double[][] {
            {cos, -sin, 0.0},
            {sin, cos, 0.0},
            {0.0, 0.0, 1.0}
        });
    }

    /**
     * Returns an affine translation matrix of 3x3 size
     * @param x x translation
     * @param y y translation
     * @return Matrix of 3x3 size
     */
    public static Matrix translationTransform(double x, double y) {
        return new Matrix(new double[][] {
            {1, 0, x},
            {0, 1, y},
            {0, 0, 1}
        });
    }

    /**
     * Returns an affine transformation of 3x3 matrix. This matrix represents a rotation and then a translation
     * @param x x translation
     * @param y y translation
     * @param angle radians; + = CCW, - = CW
     * @return 3x3 transformation matrix
     */
    public static Matrix createTransformation(double x, double y, double angle) {
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        return new Matrix(new double[][] {
            {cos, -sin, x},
            {sin, cos, y},
            {0.0, 0.0, 1.0}
        });
    }

    public static Matrix fromRows(Vector... rows) {
        double[][] vals = new double[rows.length][rows[0].size()];

        for (int i = 0; i < vals.length; i++) {
            vals[i] = rows[i].elements();
        }

        return new Matrix(vals);
    }

    public static Matrix fromCols(Vector... cols) {
        double[][] vals = new double[cols[0].size()][cols.length];

        for (int i = 0; i < vals.length; i++) {
            for (int j = 0; j < vals[0].length; j++) {
                vals[i][j] = cols[j].get(i);
            }
        }

        return new Matrix(vals);
    }

    /**
     * Gets the value at a specific coordinate.
     * * @param r Row index (0-based).
     *
     * @param c Column index (0-based).
     * @return The value at the specified position.
     */
    public double get(int r, int c) {
        return data[r * cols + c];
    }

    /**
     * Performs matrix addition.
     * * @param other The matrix to add to this one.
     *
     * @return A new Matrix representing the sum.
     * @throws IllegalArgumentException if dimensions do not match.
     */
    public Matrix plus(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols)
            throw new IllegalArgumentException("Matrix dimensions must match for addition.");
        double[][] data = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                data[i][j] = get(i, j) + other.get(i, j);
            }
        }
        return new Matrix(data);
    }

    /**
     * Performs matrix subtraction.
     * * @param other The matrix to subtract from this one.
     *
     * @return A new Matrix representing the sum.
     * @throws IllegalArgumentException if dimensions do not match.
     */
    public Matrix minus(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols)
            throw new IllegalArgumentException("Matrix dimensions must match for subtraction.");
        double[][] data = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                data[i][j] = get(i, j) - other.get(i, j);
            }
        }
        return new Matrix(data);
    }

    /**
     * Performs matrix multiplication (Dot Product).
     * * @param other The matrix to multiply by.
     *
     * @return A new Matrix representing the product.
     * @throws IllegalArgumentException if this.cols != other.rows.
     */
    public Matrix times(Matrix other) {
        if (this.cols != other.rows)
            throw new IllegalArgumentException("Dimensions mismatch: Columns of A must equal Rows of B.");
        double[][] data = new double[this.rows][other.cols];
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                for (int k = 0; k < this.cols; k++) {
                    data[i][j] += get(i, k) * other.get(k, j);
                }
            }
        }
        return new Matrix(data);
    }

    /**
     * Creates a new matrix that is the transpose of the current matrix.
     * * @return A new Matrix where rows and columns are swapped.
     */
    public Matrix transpose() {
        double[][] data = new double[cols][rows];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                data[j][i] = get(i, j);
            }
        }
        return new Matrix(data);
    }

    /**
     * Multiplies this matrix by any n-dimensional Vector.
     */
    public Vector times(Vector v) {
        if (this.cols != v.size()) throw new IllegalArgumentException("Dimension mismatch");

        double[] result = new double[this.rows];
        for (int i = 0; i < this.rows; i++) {
            double sum = 0;
            for (int j = 0; j < this.cols; j++) {
                sum += this.get(i, j) * v.get(j);
            }
            result[i] = sum;
        }
        return new Vector(result);
    }

    public double[] getDiagonal() {
        double[] elements = new double[Math.min(rows, cols)];
        for (int i = 0; i < elements.length; i++) {
            elements[i] = get(i, i);
        }
        return elements;
    }

    public double[] getRow(int i) {
        if (i >= rows) throw new IllegalArgumentException("Row index out of bounds");
        double[] vals = new double[cols];

        for (int j = 0; j < cols; j++) vals[j] = get(i, j);

        return vals;
    }

    public double[] getCol(int i) {
        if (i >= cols) throw new IllegalArgumentException("Col index out of bounds");
        double[] vals = new double[rows];

        for (int j = 0; j < rows; j++) vals[j] = get(j, i);

        return vals;
    }

    public int numRows() {
        return rows;
    }

    public int numCols() {
        return cols;
    }

    public Vector[] getRows() {
        Vector[] rows = new Vector[this.rows];

        for (int j = 0; j < rows.length; j++) rows[j] = new Vector(getRow(j));

        return rows;
    }

    public Vector[] getCols() {
        Vector[] cols = new Vector[this.cols];

        for (int j = 0; j < cols.length; j++) cols[j] = new Vector(getCol(j));

        return cols;
    }

    public Matrix clampDiagonals(double epsilon) {
        double[][] data = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (i != j) data[i][j] = get(i, j);
                else data[i][j] = Math.max(epsilon, get(i, j));
            }
        }

        return new Matrix(data);
    }

    private int isPivotInCol(int startRow, int col) {
        int best = -1;
        double max = 0.0;

        for (int r = startRow; r < rows; r++) {
            double v = Math.abs(get(r, col));
            if (v > max) {
                max = v;
                best = r;
            }
        }
        return max == 0.0 ? -1 : best;
    }

    public Matrix rowSwap(int srcRow, int destRow) {
        double[][] data = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            data[i] = getRow(i);
        }

        double[] temp = data[srcRow];
        data[srcRow] = data[destRow];
        data[destRow] = temp;

        return new Matrix(data);
    }

    public Matrix rowScale(int row, double scalar) {
        double[][] data = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            if (i != row) data[i] = getRow(i);
            else {
                for (int j = 0; j < cols; j++) data[i][j] = get(i, j) * scalar;
            }
        }

        return new Matrix(data);
    }

    public Matrix rowAdd(int srcRow, int destRow, double scalar) {
        double[][] data = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            if (i == destRow) {
                for (int j = 0; j < cols; j++) {
                    data[i][j] = get(destRow, j) + get(srcRow, j) * scalar;
                }
            } else {
                data[i] = getRow(i);
            }
        }

        return new Matrix(data);
    }

    public Pair<Matrix, Matrix> rref(Matrix augment) {
        int leadCol = 0;
        Matrix A = this;
        Matrix B = augment;

        // Forward elimination
        for (int r = 0; r < rows && leadCol < cols; r++) {

            int pivot = isPivotInCol(r, leadCol);
            while (pivot == -1) {
                leadCol++;
                if (leadCol >= cols) {
                    return Pair.of(A, B);
                }
                pivot = isPivotInCol(r, leadCol);
            }

            A = A.rowSwap(r, pivot);
            B = B.rowSwap(r, pivot);

            for (int r2 = r + 1; r2 < rows; r2++) {
                if (A.get(r2, leadCol) != 0.0) {
                    double scalar = -A.get(r2, leadCol) / A.get(r, leadCol);
                    A = A.rowAdd(r, r2, scalar);
                    B = B.rowAdd(r, r2, scalar);
                }
            }

            leadCol++;
        }

        // Normalize pivot rows
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                if (A.get(r, c) != 0.0) {
                    double inv = 1.0 / A.get(r, c);
                    A = A.rowScale(r, inv);
                    B = B.rowScale(r, inv);
                    break;
                }
            }
        }

        // Back substitution (FIXED LOOP BOUND)
        for (int r = rows - 1; r >= 0; r--) {
            int pivotCol = -1;
            for (int c = 0; c < cols; c++) {
                if (A.get(r, c) != 0.0) {
                    pivotCol = c;
                    break;
                }
            }

            if (pivotCol != -1) {
                for (int r2 = 0; r2 < r; r2++) {
                    double scalar = -A.get(r2, pivotCol);
                    A = A.rowAdd(r, r2, scalar);
                    B = B.rowAdd(r, r2, scalar);
                }
            }
        }

        return Pair.of(A, B);
    }

    public Vector solve(Vector constants) {
        return new Vector(rref(constants.toMatrix()).second().getCol(0));
    }

    public Matrix solve(Matrix augment) {
        return rref(augment).second();
    }

    /**
     * Returns the minor of this matrix formed by removing the specified row
     * and column.
     *
     * @param skipRow the row to exclude
     * @param skipCol the column to exclude
     * @return the submatrix with the given row and column removed
     */
    private Matrix minor(int skipRow, int skipCol) {
        double[][] minor = new double[rows - 1][cols - 1];
        int r = 0;

        for (int i = 0; i < rows; i++) {
            if (i == skipRow) continue;
            int c = 0;
            for (int j = 0; j < cols; j++) {
                if (j == skipCol) continue;
                minor[r][c++] = get(i, j);
            }
            r++;
        }
        return new Matrix(minor);
    }

    /**
     * Computes the determinant of the matrix using Laplace Expansion
     * @precondition The matrix must be a square matrix
     * @return the determinant of the matrix
     */
    public double determinant() {
        if (rows != cols) throw new IllegalStateException("Determinant only defined for square matrices");

        switch (rows) {
            case 0:
                return 1;
            case 1:
                return get(0, 0);
            case 2:
                return get(0, 0) * get(1, 1) - get(0, 1) * get(1, 0);
        }

        double det = 0;
        for (int j = 0; j < cols; j++)
            det += ((j % 2 == 0) ? 1 : -1) * get(0, j) * minor(0, j).determinant();
        return det;
    }

    /**
     * Gets the inverse matrix
     * @param matrix An invertible 2x2 matrix to compute the inverse of
     * @return the inverse of the given matrix
     */
    public static Matrix inverse2x2(Matrix matrix) {
        if (matrix.rows != 2 || matrix.cols != 2) throw new IllegalArgumentException("Matrix is not 2x2");

        double det = matrix.determinant();
        if (det == 0.0) throw new IllegalArgumentException("Matrix is singular");

        double[][] inv = new double[2][2];

        inv[0][0] = matrix.get(1, 1) / det;
        inv[0][1] = -matrix.get(0, 1) / det;
        inv[1][0] = -matrix.get(1, 0) / det;
        inv[1][1] = matrix.get(0, 0) / det;

        return new Matrix(inv);
    }

    /**
     * Gets the inverse matrix
     * @param matrix An invertible 3x3 matrix to compute the inverse of
     * @return the inverse of the given matrix
     */
    public static Matrix inverse3x3(Matrix matrix) {
        if (matrix.rows != 3 || matrix.cols != 3) throw new IllegalArgumentException("Matrix is not 3x3");

        double det = matrix.determinant();
        if (det == 0.0) throw new IllegalArgumentException("Matrix is singular");

        double[][] inv = new double[3][3];

        inv[0][0] = (matrix.get(1, 1) * matrix.get(2, 2) - matrix.get(1, 2) * matrix.get(2, 1)) / det;
        inv[0][1] = -(matrix.get(0, 1) * matrix.get(2, 2) - matrix.get(0, 2) * matrix.get(2, 1)) / det;
        inv[0][2] = (matrix.get(0, 1) * matrix.get(1, 2) - matrix.get(0, 2) * matrix.get(1, 1)) / det;

        inv[1][0] = -(matrix.get(1, 0) * matrix.get(2, 2) - matrix.get(1, 2) * matrix.get(2, 0)) / det;
        inv[1][1] = (matrix.get(0, 0) * matrix.get(2, 2) - matrix.get(0, 2) * matrix.get(2, 0)) / det;
        inv[1][2] = -(matrix.get(0, 0) * matrix.get(1, 2) - matrix.get(0, 2) * matrix.get(1, 0)) / det;

        inv[2][0] = (matrix.get(1, 0) * matrix.get(2, 1) - matrix.get(1, 1) * matrix.get(2, 0)) / det;
        inv[2][1] = -(matrix.get(0, 0) * matrix.get(2, 1) - matrix.get(0, 1) * matrix.get(2, 0)) / det;
        inv[2][2] = (matrix.get(0, 0) * matrix.get(1, 1) - matrix.get(0, 1) * matrix.get(1, 0)) / det;

        return new Matrix(inv);
    }

    public Matrix invert() {
        if (rows != cols) throw new IllegalStateException("Matrix must be square");

        if (rows == 1) {
            if (get(0, 0) == 0.0) throw new IllegalArgumentException("Matrix is singular");
            else return new Matrix(new double[][] {{1 / get(0, 0)}});
        }
        if (rows == 2) return Matrix.inverse2x2(this);
        if (rows == 3) return Matrix.inverse3x3(this);

        Matrix[] PLU = PTLUDecomposition();
        Matrix Y = forwardSubstitute(PLU[1], PLU[0]);
        return backSubstitute(PLU[2], Y);
    }

    public static Vector backSubstitute(Matrix A, Vector b) {
        if (A.rows != A.cols || A.rows != b.size()) {
            throw new IllegalArgumentException("Dimension mismatch");
        }

        double[] x = new double[b.size()];

        for (int i = b.size() - 1; i >= 0; i--) {
            double sum = b.get(i);

            for (int j = i + 1; j < b.size(); j++) {
                sum -= A.get(i, j) * x[j];
            }

            if (Math.abs(A.get(i, i)) < 1e-7) {
                throw new ArithmeticException("Matrix is singular on diagonal");
            }

            x[i] = sum / A.get(i, i);
        }

        return new Vector(x);
    }

    public static Matrix backSubstitute(Matrix A, Matrix B) {
        if (A.cols != A.rows || B.rows != A.rows) {
            throw new IllegalArgumentException("Dimension mismatch");
        }

        double[][] X = new double[A.rows][B.cols];
        for (int i = A.rows - 1; i >= 0; i--) {
            if (Math.abs(A.get(i, i)) < 1e-7) {
                throw new ArithmeticException("Matrix is singular on diagonal");
            }

            for (int j = 0; j < B.cols; j++) {
                double sum = B.get(i, j);
                for (int k = i + 1; k < A.rows; k++) {
                    sum -= A.get(i, k) * X[k][j];
                }
                X[i][j] = sum / A.get(i, i);
            }
        }
        return new Matrix(X);
    }

    public static Vector forwardSubstitute(Matrix A, Vector b) {
        if (A.cols != A.rows || b.size() != A.rows) {
            throw new IllegalArgumentException("Dimension mismatch");
        }

        double[] x = new double[A.rows];

        for (int i = 0; i < x.length; i++) {
            double sum = b.get(i);

            for (int k = 0; k < i; k++) {
                sum -= A.get(i, k) * x[k];
            }

            if (Math.abs(A.get(i, i)) < 1e-7) {
                throw new ArithmeticException("Matrix is singular on diagonal");
            }
            x[i] = sum / A.get(i, i);
        }

        return new Vector(x);
    }

    public static Matrix forwardSubstitute(Matrix A, Matrix B) {
        if (A.cols != A.rows || B.rows != A.rows) {
            throw new IllegalArgumentException("Dimension mismatch");
        }

        double[][] X = new double[A.rows][B.cols];

        for (int i = 0; i < A.rows; i++) {
            if (Math.abs(A.get(i, i)) < 1e-7) {
                throw new ArithmeticException("Matrix is singular on diagonal");
            }

            for (int j = 0; j < B.cols; j++) {
                double sum = B.get(i, j);
                for (int k = 0; k < i; k++) {
                    sum -= A.get(i, k) * X[k][j];
                }
                X[i][j] = sum / A.get(i, i);
            }
        }

        return new Matrix(X);
    }

    public Pair<Matrix, Matrix> QRFactorization() {
        Vector[] colVectors = getCols();
        Vector[] orthogonalBasis = Vector.gramSchmidt(colVectors);

        double[][] Q_vals = new double[rows][cols];
        for (int j = 0; j < cols; j++) {
            for (int i = 0; i < rows; i++) {
                Q_vals[i][j] = orthogonalBasis[j].get(i);
            }
        }
        Matrix Q = new Matrix(Q_vals);

        double[][] R_vals = new double[cols][cols];
        for (int i = 0; i < cols; i++) {
            Vector q_i = new Vector(Q.getCol(i));

            for (int j = i; j < cols; j++) {
                Vector a_j = new Vector(getCol(j));
                R_vals[i][j] = q_i.dot(a_j);
            }
        }
        Matrix R = new Matrix(R_vals);

        return Pair.of(Q, R);
    }

    public Matrix choleskyDecomposition() {
        if (rows != cols) {
            throw new IllegalArgumentException("Cholesky decomposition requires a square matrix.");
        }

        double[][] L_vals = new double[rows][rows];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j <= i; j++) {
                double sum = get(i, j);

                for (int k = 0; k < j; k++) {
                    sum -= L_vals[i][k] * L_vals[j][k];
                }

                if (i == j) {
                    if (sum <= 0) {
                        throw new IllegalArgumentException(
                                "Cannot use Cholesky decomposition on matrix that isn't positive-definite.");
                    }

                    L_vals[i][j] = Math.sqrt(sum);
                } else {
                    L_vals[i][j] = sum / L_vals[j][j];
                }
            }
        }

        return new Matrix(L_vals);
    }

    public Matrix[] PTLUDecomposition() {
        if (rows != cols) {
            throw new IllegalArgumentException("LU Decomposition requires a square matrix");
        }

        Matrix U = new Matrix(this.data, rows, cols);
        Matrix P = Matrix.identity(rows);
        double[][] L_vals = new double[rows][rows];

        for (int r = 0; r < rows; r++) {
            int pivotRow = r;
            double maxVal = Math.abs(U.get(r, r));

            for (int i = r + 1; i < rows; i++) {
                double absVal = Math.abs(U.get(i, r));
                if (absVal > maxVal) {
                    maxVal = absVal;
                    pivotRow = i;
                }
            }

            if (maxVal < 1e-7) {
                throw new ArithmeticException("Matrix is singular or near-singular");
            }

            if (pivotRow != r) {
                U = U.rowSwap(r, pivotRow);
                P = P.rowSwap(r, pivotRow);

                for (int k = 0; k < r; k++) {
                    double temp = L_vals[r][k];
                    L_vals[r][k] = L_vals[pivotRow][k];
                    L_vals[pivotRow][k] = temp;
                }
            }

            double pivotVal = U.get(r, r);

            for (int r2 = r + 1; r2 < rows; r2++) {
                if (U.get(r2, r) != 0.0) {
                    double scalar = -U.get(r2, r) / pivotVal;
                    U = U.rowAdd(r, r2, scalar);
                    L_vals[r2][r] = -scalar;
                }
            }
        }

        for (int i = 0; i < rows; i++) {
            L_vals[i][i] = 1.0;
        }

        Matrix L = new Matrix(L_vals);
        return new Matrix[] {P, L, U};
    }

    /**
     * Build a string that represents the elements of the matrix
     * @return String obj
     */
    @Override
    public String toString() {
        if (rows == 0 || cols == 0) return "[]";
        StringBuilder builder = new StringBuilder("[");
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                builder.append(String.format(Locale.getDefault(), "%.5f, ", get(i, j)));
            }
            builder.setLength(builder.length() - 2);
            builder.append("; ");
        }
        builder.setLength(builder.length() - 2);
        builder.append("]");
        return builder.toString();
    }

    /**
     * Checks whether this matrix is approximately equal to another matrix.
     * Equality is determined element-wise within a given tolerance.
     *
     * @param other the matrix to compare against
     * @param eps numerical tolerance
     * @return true if matrices are equal within tolerance
     */
    public boolean equals(Matrix other, double eps) {
        if (other == null) return false;
        if (this.rows != other.rows || this.cols != other.cols) return false;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (Math.abs(get(i, j) - other.get(i, j)) > eps) return false;
            }
        }
        return true;
    }

    @Override
    public boolean equals(Object other) {
        if (!(other instanceof Matrix)) return false;
        return equals((Matrix) other, 1e-9);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(data);
    }
}
