package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import java.util.Arrays;

/**
 * This is the Matrix class. This defines matrices, primarily for use in the localizers. However, if
 * matrices and matrix operations are necessary, this class as well as some operations in the
 * MathFunctions class can absolutely be used there as well. It's similar to Mats in OpenCV if you've
 * used them before, but with more limited functionality.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
public class Matrix {
    private double[][] matrix;

    /**
     * This creates a new Matrix of width and height 0.
     */
    public Matrix() {
        matrix = new double[0][0];
    }

    /**
     * This creates a new Matrix with a specified width and height.
     *
     * @param rows the number of rows, or height
     * @param columns the number of columns, or width
     */
    public Matrix(int rows, int columns) {
        matrix = new double[rows][columns];
    }

    /**
     * This creates a new Matrix from a 2D matrix of doubles. Please only enter rectangular 2D
     * Arrays of doubles or else things mess up.
     *
     * @param setMatrix the 2D Array of doubles
     */
    public Matrix(double[][] setMatrix) {
        setMatrix(setMatrix);
    }

    /**
     * This creates a new Matrix from another Matrix.
     *
     * @param setMatrix the Matrix input.
     */
    public Matrix(Matrix setMatrix) {
        setMatrix(setMatrix);
    }

    /**
     * This creates a copy of a 2D Array of doubles that references entirely new memory locations
     * from the original 2D Array of doubles, so no issues with mutability.
     *
     * @param copyMatrix the 2D Array of doubles to copy
     * @return returns a deep copy of the input Array
     */
    public static double[][] deepCopy(double[][] copyMatrix) {
        double[][] returnMatrix = new double[copyMatrix.length][copyMatrix[0].length];
        for (int i = 0; i < copyMatrix.length; i++) {
            returnMatrix[i] = Arrays.copyOf(copyMatrix[i], copyMatrix[i].length);
        }
        return returnMatrix;
    }

    /**
     * This returns a deep copy of the 2D Array that this Matrix is based on.
     *
     * @return returns the 2D Array of doubles this Matrix is built on
     */
    public double[][] getMatrix() {
        return deepCopy(matrix);
    }

    /**
     * This returns a specified row of the Matrix in the form of an Array of doubles.
     *
     * @param row the index of the row to return
     * @return returns the row of the Matrix specified
     */
    public double[] get(int row) {
        return Arrays.copyOf(matrix[row], matrix[row].length);
    }

    /**
     * This returns a specified element of the Matrix as a double.
     *
     * @param row the index of the row of the element
     * @param column the index of the column of the element
     * @return returns the element of the Matrix specified
     */
    public double get(int row, int column) {
        return get(row)[column];
    }

    /**
     * This returns the number of rows of the Matrix, as determined by the length of the 2D Array.
     * If the Matrix/2D Array is not rectangular, issues arise.
     *
     * @return returns the number of rows in the Matrix
     */
    public int getRows() {
        return matrix.length;
    }

    /**
     * This returns the number of columns of the Matrix, as determined by the length of the first Array
     * in the 2D Array. If the Matrix/2D Array is not rectangular, issues arise.
     *
     * @return returns the number of columns in the Matrix
     */
    public int getColumns() {
        return matrix[0].length;
    }

    /**
     * This sets the 2D Array of this Matrix to a copy of the 2D Array of another Matrix.
     *
     * @param setMatrix the Matrix to copy from
     * @return returns if the operation was successful
     */
    public boolean setMatrix(Matrix setMatrix) {
        return setMatrix(setMatrix.getMatrix());
    }

    /**
     * This sets the 2D Array of this Matrix to a copy of a specified 2D Array.
     *
     * @param setMatrix the 2D Array to copy from
     * @return returns if the operation was successful
     */
    public boolean setMatrix(double[][] setMatrix) {
        int columns = setMatrix[0].length;
        for (int i = 0; i < setMatrix.length; i++) {
            if (setMatrix[i].length != columns) {
                return false;
            }
        }
        matrix = deepCopy(setMatrix);
        return true;
    }

    /**
     * This sets a row of the Matrix to a copy of a specified Array of doubles.
     *
     * @param row the row to be written over
     * @param input the Array input
     * @return returns if the operation was successful
     */
    public boolean set(int row, double[] input) {
        if (input.length != getColumns()) {
            return false;
        }
        matrix[row] = Arrays.copyOf(input, input.length);
        return true;
    }

    /**
     * This sets a specified element of the Matrix to an input value.
     *
     * @param row the index of the row of the specified element
     * @param column the index of the column of the specified element
     * @param input the input value
     * @return returns if the operation was successful
     */
    public boolean set(int row, int column, double input) {
        matrix[row][column] = input;
        return true;
    }

    /**
     * This adds a Matrix to this Matrix.
     *
     * @param input the Matrix to add to this. Nothing will change in this Matrix
     * @return returns if the operation was successful
     */
    public boolean add(Matrix input) {
        if (input.getRows() == getRows() && input.getColumns() == getColumns()) {
            for (int i = 0; i < getRows(); i++) {
                for (int j = 0; j < getColumns(); j++) {
                    set(i, j, get(i,j) + input.get(i,j));
                }
            }
            return true;
        }
        return false;
    }

    /**
     * This subtracts a Matrix from this Matrix.
     *
     * @param input the Matrix to subtract from this. Nothing will change in this Matrix
     * @return returns if the operation was successful
     */
    public boolean subtract(Matrix input) {
        if (input.getRows() == getRows() && input.getColumns() == getColumns()) {
            for (int i = 0; i < getRows(); i++) {
                for (int j = 0; j < getColumns(); j++) {
                    set(i, j, get(i,j) - input.get(i,j));
                }
            }
            return true;
        }
        return false;
    }

    /**
     * This multiplies this Matrix with a scalar.
     *
     * @param scalar the scalar number
     * @return returns if the operation was successful
     */
    public boolean scalarMultiply(double scalar) {
        for (int i = 0; i < getRows(); i++) {
            for (int j = 0; j < getColumns(); j++) {
                set(i, j, scalar * get(i,j));
            }
        }
        return true;
    }

    /**
     * This multiplies the Matrix by -1, flipping the signs of all the elements within.
     *
     * @return returns if the operation was successful
     */
    public boolean flipSigns() {
        return scalarMultiply(-1);
    }

    /**
     * This multiplies a Matrix to this Matrix.
     *
     * @param input the Matrix to multiply to this. Nothing will change in this Matrix
     * @return returns if the operation was successful
     */
    public boolean multiply(Matrix input) {
        if (getColumns() == input.getRows()) {
            Matrix product = new Matrix(getRows(), input.getColumns());
            for (int i = 0; i < product.getRows(); i++) {
                for (int j = 0; j < product.getColumns(); j++) {
                    double value = 0;
                    for (int k = 0; k < get(i).length; k++) {
                        value += get(i, k) * input.get(k, j);
                    }
                    product.set(i, j, value);
                }
            }
            setMatrix(product);
            return true;
        }
        return false;
    }

    /**
     * This multiplies a Matrix to another Matrix. This will not change any data in the two input
     * Matrices.
     *
     * @param one the first Matrix to multiply.
     * @param two the second Matrix to multiply
     * @return returns if the operation was successful
     */
    public static Matrix multiply(Matrix one, Matrix two) {
        Matrix returnMatrix = new Matrix(one);
        if (returnMatrix.multiply(two)) {
            return returnMatrix;
        } else {
            return new Matrix();
        }
    }
}
