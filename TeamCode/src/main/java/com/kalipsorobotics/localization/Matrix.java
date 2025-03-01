package com.kalipsorobotics.localization;

public class Matrix {
    double[][] data;
    int rows;
    public int cols;

    public Matrix(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
        this.data = new double[rows][cols];
    }

    // Constructor to initialize a matrix with values
    public Matrix(double[][] data) {
        this.rows = data.length;
        this.cols = data[0].length;
        this.data = data;
    }

    // Matrix multiplication
    public Matrix multiply(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException("Matrix dimensions do not match for multiplication.");
        }
        Matrix result = new Matrix(this.rows, other.cols);
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                for (int k = 0; k < this.cols; k++) {
                    result.data[i][j] += this.data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }

    // Matrix addition
    public Matrix add(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrix dimensions do not match for addition.");
        }
        Matrix result = new Matrix(this.rows, this.cols);
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < this.cols; j++) {
                result.data[i][j] = this.data[i][j] + other.data[i][j];
            }
        }
        return result;
    }

    // Matrix subtraction
    public Matrix subtract(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrix dimensions do not match for subtraction.");
        }
        Matrix result = new Matrix(this.rows, this.cols);
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < this.cols; j++) {
                result.data[i][j] = this.data[i][j] - other.data[i][j];
            }
        }
        return result;
    }

    // Matrix transpose
    public Matrix transpose() {
        Matrix result = new Matrix(this.cols, this.rows);
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < this.cols; j++) {
                result.data[j][i] = this.data[i][j];
            }
        }
        return result;
    }

    // Inverse of the matrix (using Gaussian elimination or another method)
    public Matrix inverse() {
        if (this.rows != this.cols) {
            throw new IllegalArgumentException("Matrix must be square for inversion.");
        }

        int n = this.rows;
        Matrix augmented = new Matrix(n, 2 * n);
        // Create the augmented matrix [A | I]
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                augmented.data[i][j] = this.data[i][j];
                augmented.data[i][j + n] = (i == j) ? 1 : 0;
            }
        }

        // Apply Gaussian elimination
        for (int i = 0; i < n; i++) {
            // Find the row with the largest element in column i
            int maxRow = i;
            for (int j = i + 1; j < n; j++) {
                if (Math.abs(augmented.data[j][i]) > Math.abs(augmented.data[maxRow][i])) {
                    maxRow = j;
                }
            }

            // Swap rows
            double[] temp = augmented.data[i];
            augmented.data[i] = augmented.data[maxRow];
            augmented.data[maxRow] = temp;

            // Make the diagonal element 1
            double diag = augmented.data[i][i];
            for (int j = 0; j < 2 * n; j++) {
                augmented.data[i][j] /= diag;
            }

            // Eliminate column elements below the diagonal
            for (int j = i + 1; j < n; j++) {
                double factor = augmented.data[j][i];
                for (int k = 0; k < 2 * n; k++) {
                    augmented.data[j][k] -= factor * augmented.data[i][k];
                }
            }
        }

        // Back substitution to zero out the upper triangular elements
        for (int i = n - 1; i >= 0; i--) {
            for (int j = i - 1; j >= 0; j--) {
                double factor = augmented.data[j][i];
                for (int k = 0; k < 2 * n; k++) {
                    augmented.data[j][k] -= factor * augmented.data[i][k];
                }
            }
        }

        // Extract the inverse matrix from the augmented matrix
        Matrix inverse = new Matrix(n, n);
        for (int i = 0; i < n; i++) {
            System.arraycopy(augmented.data[i], n, inverse.data[i], 0, n);
        }

        return inverse;
    }

    // Return a string representation of the matrix
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                sb.append(String.format("%.2f ", data[i][j]));
            }
            sb.append("\n");
        }
        return sb.toString();
    }
}
