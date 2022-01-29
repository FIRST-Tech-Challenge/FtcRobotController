package com.SCHSRobotics.HAL9001.util.math.geometry;

import static java.lang.Math.abs;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.pow;

import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.math.FakeNumpy;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;


/**
 * A matrix class for doing matrix 
 *
 * Creation Date: 5/27/20
 *
 * @author Cole Savage, Level Up
 * @since 1.1.0
 * @version 1.0.0
 *
 * @see MatrixSimple
 */
public class MatrixSimple {

    protected static final int DECIMAL_ACCURACY = 7;

    //The double array matrix of values contained in the matrix.
    protected final double[][] vals;


    /**
     * A constructor for matrix where individual arrays of doubles can be entered or a matrix of doubles can be entered.
     *
     * @param vals The list of rows in the matrix.
     * @throws RuntimeException Throws this exception if the entered rows are not all of the same length.
     */
    public MatrixSimple(double[]... vals) {
        if(vals == null) {
            this.vals = new double[1][1];
            return;
        }

        for(double[] row : vals) ExceptionChecker.assertTrue(row.length == vals[0].length, new RuntimeException("Entered rows in matrix must all be the same length."));
        this.vals = vals.clone();
    }

    /**
     * A private constructor for matrix used for cloning.
     *
     * @param matrix The matrix to clone.
     */
    private MatrixSimple(@NotNull MatrixSimple matrix) {
        vals = new double[matrix.getNumRows()][matrix.getNumCols()];
        for (int i = 0; i < matrix.getNumRows(); i++) System.arraycopy(matrix.vals[i], 0, vals[i], 0, vals[i].length);
    }

    public MatrixSimple copy() {
        return new MatrixSimple(this);
    }

    /**
     * Sets an entry in the matrix at the given row and column to the desired value.
     *
     * @param row The row index of the entry to set.
     * @param col The column index of the entry to set.
     * @param value The value to set the entry in the matrix at (row, column) to.
     * @throws RuntimeException Throws this exception if the row index isn't within the range [0, getNumRows()) or the column index isn't within the range [0, getNumCols()).
     */
    public void set(int row, int col, double value) {
        ExceptionChecker.assertTrue(row < getNumRows(), new RuntimeException("Cannot set an entry in a row outside the matrix."));
        ExceptionChecker.assertTrue(row >= 0, new RuntimeException("Cannot set an entry in a negative row."));
        ExceptionChecker.assertTrue(col < getNumCols(), new RuntimeException("Cannot set an entry in a column outside the matrix."));
        ExceptionChecker.assertTrue(col >= 0, new RuntimeException("Cannot set an entry in a negative column."));

        vals[row][col] = value;
    }

    /**
     * Gets the value in the matrix at the specified row, column entry.
     *
     * @param row The row index.
     * @param col The column index.
     * @return The entry at the given row and column in the matrix.
     * @throws RuntimeException Throws this exception if the row index isn't within the range [0, getNumRows()) or the column index isn't within the range [0, getNumCols()).
     */
    public double get(int row, int col) {
        ExceptionChecker.assertTrue(row < getNumRows(), new RuntimeException("Cannot get an entry in a row outside the matrix."));
        ExceptionChecker.assertTrue(row >= 0, new RuntimeException("Cannot get an entry in a negative row."));
        ExceptionChecker.assertTrue(col < getNumCols(), new RuntimeException("Cannot get an entry in a column outside the matrix."));
        ExceptionChecker.assertTrue(col >= 0, new RuntimeException("Cannot get an entry in a negative column."));

        return vals[row][col];
    }

    /**
     * Gets the row at the specified row index in the matrix.
     *
     * @param row The row index.
     * @return The row at the given row index.
     * @throws RuntimeException Throws this exception if the row index isn't within the range [0, getNumRows()).
     */
    public double[] getRowArray(int row) {
        ExceptionChecker.assertTrue(row < getNumRows(), new RuntimeException("Cannot get an entry in a row outside the matrix."));
        ExceptionChecker.assertTrue(row >= 0, new RuntimeException("Cannot get an entry in a negative row."));

        return vals[row].clone();
    }

    /**
     * Gets the column at the specified column index in the matrix.
     *
     * @param col The column index.
     * @return The column at the given column index.
     * @throws RuntimeException Throws this exception if the row index isn't within the range [0, getNumCols()).
     */
    public double[] getColArray(int col) {
        ExceptionChecker.assertTrue(col < getNumCols(), new RuntimeException("Cannot get an entry in a column outside the matrix."));
        ExceptionChecker.assertTrue(col >= 0, new RuntimeException("Cannot get an entry in a negative column."));

        double[] colArray = new double[getNumRows()];
        for (int row = 0; row < getNumRows(); row++) colArray[row] = get(row, col);
        return colArray;
    }

    public MatrixSimple getCol(int col) {
        ExceptionChecker.assertTrue(col < getNumCols(), new RuntimeException("Cannot get an entry in a column outside the matrix."));
        ExceptionChecker.assertTrue(col >= 0, new RuntimeException("Cannot get an entry in a negative column."));

        MatrixSimple column = MatrixSimple.zeroMatrix(getNumRows(), 1);
        for (int row = 0; row < getNumRows(); row++) column.set(row, 0, get(row, col));
        return column;
    }

    public MatrixSimple getRow(int row) {
        return new MatrixSimple(getRowArray(row));
    }

    public MatrixSimple[] getCols() {
        MatrixSimple[] columns = new MatrixSimple[getNumCols()];
        for (int col = 0; col < getNumCols(); col++) {
            columns[col] = getCol(col);
        }
        return columns;
    }

    public MatrixSimple[] getRows() {
        MatrixSimple[] rows = new MatrixSimple[getNumRows()];
        for (int row = 0; row < getNumRows(); row++) {
            rows[row] = getRow(row);
        }
        return rows;
    }

    /**
     * Gets the total number of rows in the matrix.
     *
     * @return The total number of rows in the matrix.
     */
    public int getNumRows() {
        return vals.length;
    }

    /**
     * Gets the total number of columns in the matrix.
     *
     * @return The total number of columns in the matrix.
     */
    public int getNumCols() {
        if (this.getNumRows() == 0) return 0;
        return vals[0].length;
    }

    /**
     * Sets the row at the given row index to the given row.
     *
     * @param rowNum The row index.
     * @param row The row to add at that row index.
     * @throws RuntimeException Throws this exception if the length of the new row does not match the length of the old row.
     */
    public void setRowArray(int rowNum, double @NotNull [] row) {
        ExceptionChecker.assertTrue(row.length == getNumCols(), new RuntimeException("New row length does not match matrix row length."));
        vals[rowNum] = row;
    }

    public void setRow(int rowNum, @NotNull MatrixSimple row) {
        ExceptionChecker.assertTrue(row.getNumRows() == 1, new ArithmeticException("Matrix is not a row matrix."));
        setRowArray(rowNum, row.getRowArray(0));
    }

    /**
     * Sets the column at the given column index to the given column.
     *
     * @param colNum The column index.
     * @param col The column to add at that column index.
     * @throws RuntimeException Throws this exception if the length of the new column does not match the length of the old column.
     */
    public void setColArray(int colNum, double @NotNull [] col) {
        ExceptionChecker.assertTrue(col.length == getNumRows(), new RuntimeException("New column length does not match matrix column length."));
        for (int i = 0; i < getNumRows(); i++) set(i, colNum, col[i]);
    }

    public void setCol(int colNum, @NotNull MatrixSimple col) {
        ExceptionChecker.assertTrue(col.isVector(), new ArithmeticException("Matrix is not a column matrix."));
        setRowArray(colNum, col.getColArray(0));
    }

    /**
     * Gets whether the matrix is the zero matrix.
     *
     * @return Whether the matrix is the zero matrix.
     */
    public boolean isZeroMatrix() {
        int rows = getNumRows();
        int cols = getNumCols();

        boolean isZeroMatrix = true;
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                isZeroMatrix &= HALMathUtil.round(get(row, col), DECIMAL_ACCURACY) == 0;
            }
        }
        return isZeroMatrix;
    }

    /**
     * Gets whether the matrix is the identity matrix.
     *
     * @return Whether the matrix is the identity matrix.
     */
    public boolean isIdentityMatrix() {
        if (!isSquare()) return false;

        int rows = getNumRows();
        int cols = getNumCols();

        boolean isIdentity = true;
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                double val = get(row, col);

                if (row == col) isIdentity &= HALMathUtil.round(val, DECIMAL_ACCURACY) == 1;
                else isIdentity &= HALMathUtil.round(val, DECIMAL_ACCURACY) == 0;
            }
        }
        return isIdentity;
    }

    /**
     * Gets whether the matrix is a square matrix.
     *
     * @return Whether the matrix is a square matrix.
     */
    public boolean isSquare() {
        return this.getNumRows() == this.getNumCols();
    }

    /**
     * Gets whether the matrix is a diagonal matrix.
     *
     * @return Whether the matrix is a diagonal matrix.
     */
    public boolean isDiagonal() {
        ExceptionChecker.assertTrue(isSquare(), new ArithmeticException("Matrix is not square."));
        for (int row = 0; row < getNumRows(); row++) {
            for (int col = 0; col < getNumCols(); col++) {
                if(row != col && HALMathUtil.round(get(row, col), DECIMAL_ACCURACY) != 0) {
                    return false;
                }
            }
        }
        return  true;
    }

    /**
     * Gets whether the matrix is symmetric.
     *
     * @return Whether the matrix is symmetric.
     */
    public boolean isSymmetric() {
        return copy().transpose().equals(this);
    }

    /**
     * Calculates the transpose of the matrix.
     *
     * @return This matrix.
     */
    public MatrixSimple transpose() {
        int rows = getNumRows();
        int cols = getNumCols();

        double[][] matrixTranspose = new double[cols][rows];
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                matrixTranspose[col][row] = vals[row][col];
            }
        }
        return new MatrixSimple(matrixTranspose);
    }

    /**
     * Calculates the trace of this matrix.
     *
     * @return The trace of this matrix.
     */
    public double trace() {
        ExceptionChecker.assertTrue(isSquare(), new RuntimeException("Matrix is not square"));

        double trace = 0;
        for (int diag = 0; diag < getNumRows(); diag++) {
            trace += vals[diag][diag];
        }
        return trace;
    }

    public MatrixSimple augment(@NotNull MatrixSimple matrix) {
        double[][] newVals = new double[max(getNumRows(),matrix.getNumRows())][getNumCols()+matrix.getNumCols()];
        for (int row = 0; row < getNumRows(); row++) {
            if (getNumCols() >= 0) System.arraycopy(vals[row], 0, newVals[row], 0, getNumCols());
        }

        for (int row = 0; row < matrix.getNumRows(); row++) {
            for (int col = 0; col < matrix.getNumCols(); col++) {
                newVals[row][getNumCols()+col] = matrix.vals[row][col];
            }
        }

        return new MatrixSimple(newVals);
    }

    public MatrixSimple augmentVertical(@NotNull MatrixSimple matrix) {
        double[][] newVals = new double[getNumRows()+matrix.getNumRows()][max(getNumCols(),matrix.getNumCols())];

        for (int row = 0; row < getNumRows(); row++) {
            if (getNumCols() >= 0) System.arraycopy(vals[row], 0, newVals[row], 0, getNumCols());
        }

        for (int row = 0; row < matrix.getNumRows(); row++) {
            for (int col = 0; col < matrix.getNumCols(); col++) {
                newVals[getNumRows()+row][col] = matrix.vals[row][col];
            }
        }

        return new MatrixSimple(newVals);
    }

    /**
     * Calculates the determinant of this matrix.
     *
     * @return The determinant of this matrix.
     * @throws RuntimeException Throws this exception when the matrix is not square.
     */
    public double determinant() {
        ExceptionChecker.assertTrue(isSquare(), new RuntimeException("Is not a square matrix"));
        LUDecomposition lu = LUDecomposition();
        int size = getNumRows();

        double det = 1;
        for (int diag = 0; diag < size; diag++) {
            if(lu.P.get(diag, diag) != 1) det *= -1;
            det *= lu.L.get(diag, diag);
            det *= lu.U.get(diag, diag);
        }

        return det;
    }

    public MatrixSimple invert() {
        ExceptionChecker.assertTrue(isSquare(), new RuntimeException("Is not a square matrix"));
        MatrixSimple inverseFinderRREF = this.augment(MatrixSimple.identityMatrix(getNumRows())).rref();
        MatrixSimple potentialIdentity = inverseFinderRREF.crop(0, getNumRows(), 0, getNumCols());
        potentialIdentity = potentialIdentity.round(DECIMAL_ACCURACY);
        ExceptionChecker.assertTrue(potentialIdentity.isIdentityMatrix(), new RuntimeException("Matrix is not invertible!"));
        return inverseFinderRREF.crop(0, getNumRows(), getNumCols(), 2*getNumCols());
    }

    /**
     * Multiplies this matrix by a constant value.
     *
     * @param value The value to multiply by.
     * @return This matrix.
     */
    public MatrixSimple multiply(double value) {
        double[][] multipliedVals = new double[getNumRows()][getNumCols()];
        for (int row = 0; row < getNumRows(); row++) {
            for (int col = 0; col < getNumCols(); col++) {
                multipliedVals[row][col] = vals[row][col]*value;
            }
        }
        return new MatrixSimple(multipliedVals);
    }

    /**
     * Multiplies this matrix by another matrix.
     *
     * @param matrix The matrix to multiply by.
     * @return This matrix.
     * @throws RuntimeException Throws this exception when this matrix cannot be multiplied by the given matrix.
     */
    public MatrixSimple multiply(@NotNull MatrixSimple matrix) {
        ExceptionChecker.assertTrue(this.getNumCols() == matrix.getNumRows(), new RuntimeException("Cannot multiply given matrices, invalid dimensions."));

        double[][] multipliedMatrix = new double[getNumRows()][matrix.getNumCols()];
        for (int rowNum = 0; rowNum < getNumRows(); rowNum++) {
            double[] row = getRowArray(rowNum);
            for (int colNum = 0; colNum < matrix.getNumCols(); colNum++) {
                double[] col = matrix.getColArray(colNum);
                double dotProduct = 0;
                for (int k = 0; k < row.length; k++) {
                    dotProduct += row[k]*col[k];
                }
                multipliedMatrix[rowNum][colNum] = dotProduct;
            }
        }
        return new MatrixSimple(multipliedMatrix);
    }

    /**
     * Divides this matrix by a constant value.
     *
     * @param value The value to divide by.
     * @return This matrix.
     * @throws ArithmeticException Throws this exception if you try to divide by 0.
     */
    public MatrixSimple divide(double value) {
        ExceptionChecker.assertFalse(value == 0, new ArithmeticException("Divide by zero error."));
        return multiply(1.0 / value);
    }

    public MatrixSimple add(@NotNull MatrixSimple matrix) {
        ExceptionChecker.assertTrue(matrix.getNumCols() == this.getNumCols() && matrix.getNumRows() == this.getNumRows(), new RuntimeException("Matrices must be the same size!"));
        double[][] sumVals = new double[getNumRows()][getNumCols()];
        for (int row = 0; row < getNumRows(); row++) {
            for (int col = 0; col < getNumCols(); col++) {
                sumVals[row][col] = this.vals[row][col] + matrix.vals[row][col];
            }
        }

        return new MatrixSimple(sumVals);
    }

    public MatrixSimple subtract(@NotNull MatrixSimple matrix) {
        return this.add(matrix.multiply(-1));
    }

    /**
     * Masks the matrix, so that only entries in this matrix with corresponding entries > 0 in the mask will remain (other entries are set to 0).
     *
     * @param mask The matrix being used to mask this matrix.
     * @return This matrix.
     * @throws RuntimeException Throws this exception when the size of the mask is not the same size as this matrix.
     */
    public MatrixSimple mask(MatrixSimple mask) {
        double[][] maskedVals = new double[getNumRows()][getNumCols()];
        for (int row = 0; row < getNumRows(); row++) {
            for (int col = 0; col < getNumCols(); col++) {
                if (mask.vals[row][col] > 0) maskedVals[row][col] = vals[row][col];
            }
        }

        return new MatrixSimple(maskedVals);
    }

    public MatrixSimple crop(int fromRow, int toRow, int fromCol, int toCol) {
        double[][] croppedVals = new double[toRow-fromRow][toCol-fromCol];
        for (int row = 0; row < getNumRows(); row++) {
            for (int col = 0; col < getNumCols(); col++) {
                if((row >= fromRow && row < toRow) && (col >= fromCol && col < toCol)) croppedVals[row-fromRow][col-fromCol] = vals[row][col];
            }
        }
        return new MatrixSimple(croppedVals);
    }

    /**
     * Calculates the row echelon form of this matrix using Gaussian elimination.
     *
     * @return This matrix.
     */
    public MatrixSimple rref() {
        LUDecomposition luDecomposition = this.LUDecomposition();
        MatrixSimple rref = luDecomposition.U.round(DECIMAL_ACCURACY);

        for (int pivot = min(getNumRows(), getNumCols())-1; pivot >= 0; pivot--) {
            double pivotVal = rref.vals[pivot][pivot];

            if(HALMathUtil.round(pivotVal, DECIMAL_ACCURACY) != 0) {
                //Divide pivot row by pivot value to make the pivot 1.
                double[] pivotRow = rref.getRowArray(pivot);
                FakeNumpy.divide(pivotRow, pivotVal);
                rref.setRowArray(pivot, pivotRow);

                for (int row = 0; row < pivot; row++) {
                    double elimVal = rref.vals[row][pivot];

                    //Generate elimination matrix
                    MatrixSimple E = MatrixSimple.identityMatrix(getNumRows());
                    E.set(row, pivot, -elimVal);

                    rref = E.multiply(rref);
                }
            }
        }

        boolean reachedZeroRow = false;
        for (int row = 0; row < rref.getNumRows(); row++) {
            boolean isZeroRow = true;
            for (int col = 0; col < rref.getNumCols(); col++) {
                if(HALMathUtil.round(rref.get(row, col), DECIMAL_ACCURACY) != 0) {
                    isZeroRow = false;
                    break;
                }
            }
            reachedZeroRow |= isZeroRow;
            if(reachedZeroRow && !isZeroRow) {
                double[] nonZeroRow = rref.getRowArray(row);
                double[] zeroRow = rref.getRowArray(row-1);
                rref.setRowArray(row, zeroRow);
                rref.setRowArray(row-1, nonZeroRow);
            }
        }

        return rref;
    }

    public VectorSpace nullSpace() {
        QRDecomposition qr = transpose().QRDecomposition();
        int nullity = nullityFromQR(qr);

        if(nullity == 0) return new VectorSpace(MatrixSimple.zeroMatrix(getNumCols(), 1));

        MatrixSimple N = qr.Q.getCol(getNumCols()-nullity);
        for (int col = getNumCols()-nullity+1; col < getNumCols(); col++) {
            N = N.augment(qr.Q.getCol(col));
        }
        return new VectorSpace(N);
    }

    public VectorSpace columnSpace() {
        MatrixSimple rref = rref();
        List<MatrixSimple> columnSpaceBasis = new ArrayList<>();
        for (int row = 0; row < rref.getNumRows(); row++) {
            for (int col = 0; col < getNumCols(); col++) {
                if(HALMathUtil.round(rref.get(row, col), DECIMAL_ACCURACY) == 1) {
                    MatrixSimple basisVector = MatrixSimple.zeroMatrix(getNumRows(), 1);
                    for (int originalRow = 0; originalRow < getNumRows(); originalRow++) {
                        basisVector.set(originalRow, 0, get(originalRow, col));
                    }
                    columnSpaceBasis.add(basisVector);
                    break;
                }
            }
        }

        if(columnSpaceBasis.isEmpty()) return new VectorSpace(new MatrixSimple[] {MatrixSimple.zeroMatrix(getNumCols(), 1)});
        return new VectorSpace(columnSpaceBasis.toArray(new MatrixSimple[0]));
    }

    public VectorSpace rowSpace() {
        return transpose().columnSpace();
    }

    public VectorSpace leftNullSpace() {
        return transpose().nullSpace();
    }

    private @NotNull LUDecomposition LUDecomposition(@NotNull MatrixSimple P) {
        MatrixSimple L = MatrixSimple.identityMatrix(this.getNumRows());
        MatrixSimple U = P.multiply(this);

        for (int pivot = 0; pivot < min(getNumRows(), getNumCols()); pivot++) {
            double pivotVal = U.get(pivot, pivot);

            //Not actually a pivot :(
            if(pivotVal == 0) {

                int nonZeroRowIdx = -1;
                for (int row = pivot; row < U.getNumRows(); row++) {
                    if(U.get(row, pivot) != 0) {
                        nonZeroRowIdx = row;
                        break;
                    }
                }
                if(nonZeroRowIdx == -1) {
                    continue;
                }

                MatrixSimple permutation = MatrixSimple.identityMatrix(getNumRows());
                double[] pivotRow = permutation.getRowArray(pivot);
                double[] nonZeroRow = permutation.getRowArray(nonZeroRowIdx);

                permutation.setRowArray(pivot, nonZeroRow);
                permutation.setRowArray(nonZeroRowIdx, pivotRow);
                
                return LUDecomposition(permutation.multiply(P));
            }

            for (int row = pivot+1; row < getNumRows(); row++) {

                double elimVal = U.get(row, pivot) / pivotVal;

                //Generate elimination matrix and its inverse
                MatrixSimple E = MatrixSimple.identityMatrix(getNumRows());
                MatrixSimple Einv = MatrixSimple.identityMatrix(getNumRows());
                E.set(row, pivot, -elimVal);
                Einv.set(row, pivot, elimVal);

                U = E.multiply(U);
                L = L.multiply(Einv);
            }
        }

        return new LUDecomposition(P, L, U);
    }

    public LUDecomposition LUDecomposition() {
        return LUDecomposition(MatrixSimple.identityMatrix(this.getNumRows()));
    }

    public static @NotNull MatrixSimple givensRotationMatrix(int size, int modifyRow, int setRow, double c, double s)  {
        ExceptionChecker.assertTrue(setRow >= 1, new ArithmeticException("Givens' rotation can't cancel anything on row 0."));
        ExceptionChecker.assertTrue(modifyRow < setRow, new ArithmeticException("Givens' rotation secondary modified row must be before the main modified row."));
        MatrixSimple givensRotationMatrix = MatrixSimple.identityMatrix(size);

        givensRotationMatrix.set(modifyRow, modifyRow, c);
        givensRotationMatrix.set(setRow, modifyRow, s);
        givensRotationMatrix.set(modifyRow, setRow, -s);
        givensRotationMatrix.set(setRow, setRow, c);

        return givensRotationMatrix;
    }

    public QRDecomposition QRDecomposition() {


        int rows = getNumRows();
        int cols = getNumCols();

        MatrixSimple Q = MatrixSimple.identityMatrix(rows);
        MatrixSimple R = copy();

        for (int col = 0; col < cols; col++) {
            for (int row = rows-1; row > col; row--) {
                double a = R.get(row,col);
                if(a == 0) continue; //If the value is already canceled, we're done.

                double b = R.get(row - 1, col);

                double r = hypot(a, b);

                double c = b / r;
                double s = -a / r;

                MatrixSimple G = MatrixSimple.givensRotationMatrix(rows, row-1, row, c,s);

                R = G.multiply(R);
                Q = G.multiply(Q);
            }
        }

        Q = Q.transpose();

        return new QRDecomposition(Q, R);
    }

    public ComplexNumber[] eigenvaluesFull() {
        ExceptionChecker.assertTrue(this.isSquare(), new RuntimeException("Matrix must be square!"));
        MatrixSimple A = getEigenvalueDiagonalMatrix();

        ComplexNumber[] eigenvalues = new ComplexNumber[A.getNumCols()];
        final int cols = A.getNumCols();

        for (int diag = 0; diag < cols; diag++) {
            //Anything on the right subdiagonal?
            if(diag+1 < cols && HALMathUtil.round(A.get(diag, diag+1), DECIMAL_ACCURACY) != 0) {
                //YES THERE IS! COMPLEX NUMBERS ENGAGE!
                final double a = A.get(diag, diag);
                final double b = A.get(diag, diag+1);
                final double c = A.get(diag+1, diag);
                final double d = A.get(diag+1, diag+1);

                ComplexNumber discriminant = ComplexNumber.fromReal((a+d)*(a+d)-4*(a*d-b*c));
                ComplexNumber[] sqrtDiscriminants = ComplexNumber.safeSqrtFull(discriminant, DECIMAL_ACCURACY);

                eigenvalues[diag] = ComplexNumber.fromReal(a+d).add(sqrtDiscriminants[0]).divide(2);
                eigenvalues[diag+1] = ComplexNumber.fromReal(a+d).add(sqrtDiscriminants[1]).divide(2);

                //To skip the next line and ensure we only count 2x2 blocks.
                diag++;
            }
            else {
                //No? Fine its a normal real number. Be boring, I get it.
                eigenvalues[diag] = ComplexNumber.fromReal(A.get(diag, diag));
            }
        }

        return eigenvalues;
    }

    public double[] eigenvalues() {
        ExceptionChecker.assertTrue(this.isSquare(), new RuntimeException("Matrix must be square!"));

        MatrixSimple A = getEigenvalueDiagonalMatrix();

        ExceptionChecker.assertTrue(A.isDiagonal(), new ArithmeticException("Matrix eigenvalues are complex!"));

        double[] eigenvalues = new double[A.getNumRows()];
        for (int diag = 0; diag < A.getNumRows(); diag++) {
            eigenvalues[diag] = A.get(diag, diag);
        }
        return eigenvalues;
    }

    private MatrixSimple @NotNull [] eigenvectorsFromLambda(@NotNull MatrixSimple eigenvalues) {
        Set<Double> usedEigenvalues = new LinkedHashSet<>();
        List<MatrixSimple> eigenvectors = new ArrayList<>();
        for(int diag = 0; diag < eigenvalues.getNumRows(); diag++) {
            double lambda = eigenvalues.get(diag, diag);
            if(!usedEigenvalues.contains(lambda)) {
                MatrixSimple eigenvector = (this.subtract(MatrixSimple.identityMatrix(this.getNumRows()).multiply(lambda)))
                        .nullSpace().getBasisMatrix(); //The null space of A-(lambda)I
                eigenvectors.add(eigenvector);
            }
            usedEigenvalues.add(lambda);
        }
        return eigenvectors.toArray(new MatrixSimple[0]);
    }


    private double diagSquares(@NotNull MatrixSimple A) {
        double val = 0;
        for (int diag = 0; diag < A.getNumCols(); diag++) {
            double diagElem = A.get(diag, diag);
            val += diagElem*diagElem;
        }
        return val;
    }

    private MatrixSimple getEigenvalueDiagonalMatrix() {
        QRDecomposition qrDecomposition = this.QRDecomposition();
        MatrixSimple A = qrDecomposition.R.multiply(qrDecomposition.Q);

        MatrixSimple identity = MatrixSimple.identityMatrix(qrDecomposition.Q.getNumRows());

        double previous = Double.MAX_VALUE;
        while(abs(diagSquares(A)-previous) != 0) {
            final double s = A.get(getNumRows()-1, getNumRows()-1);
            MatrixSimple smult = MatrixSimple.identityMatrix(A.getNumRows()).multiply(s);

            previous = diagSquares(A);
            qrDecomposition = A.subtract(smult).QRDecomposition();
            A = qrDecomposition.R.multiply(qrDecomposition.Q).add(smult);

            //If matrix is already upper triangular
            if(qrDecomposition.Q.equals(identity)) {
                for (int diag = 0; diag < A.getNumRows(); diag++) {
                    identity.set(diag, diag, A.get(diag, diag));
                }
                A = identity;
                break;
            }

        }
        return A;
    }

    public MatrixSimple[] eigenvectors() {
        ExceptionChecker.assertTrue(this.isSquare(), new RuntimeException("Matrix must be square!"));
        MatrixSimple eigenvalues = getEigenvalueDiagonalMatrix();
        return eigenvectorsFromLambda(eigenvalues);
    }

    public EigenData eigen() {
        ExceptionChecker.assertTrue(this.isSquare(), new RuntimeException("Matrix must be square!"));

        ExceptionChecker.assertTrue(this.isSquare(), new RuntimeException("Matrix must be square!"));

        MatrixSimple lambda = getEigenvalueDiagonalMatrix();
        MatrixSimple[] eigenvectors = eigenvectorsFromLambda(lambda);

        ExceptionChecker.assertTrue(eigenvectors.length > 0, new ArithmeticException("Something has gone wrong, there were no eigenvectors."));

        MatrixSimple eigenbasis = eigenvectors[0];
        for (int i = 1; i < eigenvectors.length; i++) {
            eigenbasis = eigenbasis.augment(eigenvectors[i]);
        }

        return new EigenData(lambda, eigenbasis);
    }

    public boolean isDiagonalizable() {
        return eigen().eigenbasis.nullity() == 0;
    }

    public Diagonalization diagonalize() {
        ExceptionChecker.assertTrue(isSquare(), new ArithmeticException("Matrix is not square."));
        //ExceptionChecker.assertTrue(isDiagonalizable(), new ArithmeticException("Matrix is not diagonalizable."));

        EigenData eigenData = eigen();

        MatrixSimple lambda = MatrixSimple.identityMatrix(eigenData.eigenvalues.length);
        for (int diag = 0; diag < lambda.getNumRows(); diag++) {
            lambda.set(diag, diag, eigenData.eigenvalues[diag]);
        }

        return new Diagonalization(eigenData.eigenbasis, lambda);
    }

    private static int nullityFromQR(@NotNull MatrixSimple.QRDecomposition qr) {
        int rows = qr.R.getNumRows();
        int cols = qr.R.getNumCols();

        int rank = 0;
        for (int row = rows-1; row >= 0; row--) {
            boolean isZeroRow = true;
            for (int col = 0; col < cols; col++) {
                if(HALMathUtil.round(qr.R.get(row, col), DECIMAL_ACCURACY) != 0) {
                    isZeroRow = false;
                    break;
                }
            }
            if(isZeroRow) rank++;
        }

        return rank;
    }

    /**
     * Calculates the rank of the matrix.
     *
     * @return The rank of the matrix.
     */
    public int rank() {
        return getNumCols()-nullity();
    }

    /**
     * Calculates the nullity of the matrix (uses the rank-nullity theorem).
     *
     * @return The nullity of the matrix.
     */
    public int nullity() {
        return nullityFromQR(transpose().QRDecomposition());
    }

    public boolean isVector() {
        return getNumCols() == 1;
    }

    public VectorND toVector() {
        ExceptionChecker.assertTrue(isVector(), new ArithmeticException("matrix is not a vector."));
        return new VectorND(this);
    }

    public MatrixSimple round(int places) {
        ExceptionChecker.assertTrue(places >= 0, new ArithmeticException("You cannot have negative decimal places."));

        MatrixSimple rounded = copy();

        int rows = getNumRows();
        int cols = getNumCols();

        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                rounded.set(row, col, HALMathUtil.round(get(row, col), places));
            }
        }

        return rounded;
    }

    /**
     * Creates an identity matrix of the given size.
     *
     * @param size The size of the identity matrix to create (used for width and height, as identity matrices are square).
     * @return An identity matrix of the given size.
     * @throws RuntimeException Throws this exception if the given size is 1 or less, as identity matrices must be at least 2x2.
     */

    @NotNull
    public static MatrixSimple identityMatrix(int size) {
        ExceptionChecker.assertTrue(size > 0, new RuntimeException("An identity matrix must be at least 1x1"));
        double[][] identityVals = new double[size][size];
        for (int diag = 0; diag < size; diag++) {
            identityVals[diag][diag] = 1;
        }

        return new MatrixSimple(identityVals);
    }

    /**
     * Creates a zero matrix of the given size.
     *
     * @param rows The number of rows in the zero matrix.
     * @param cols The number of columns in the zero matrix.
     * @return A zero matrix of the given size.
     * @throws RuntimeException Throws this exception if the number of rows or columns given was negative.
     */
    @NotNull
    public static MatrixSimple zeroMatrix(int rows, int cols) {
        ExceptionChecker.assertTrue(rows > 0, new RuntimeException("Can't have 0 or negative amount of rows."));
        ExceptionChecker.assertTrue(cols > 0, new RuntimeException("Can't have 0 or negative amount of columns."));

        return new MatrixSimple(new double[rows][cols]);
    }

    /**
     * Creates a ones matrix of the given size.
     *
     * @param rows The number of rows in the ones matrix.
     * @param cols The number of columns in the ones matrix.
     * @return A ones matrix of the given size.
     * @throws RuntimeException Throws this exception if the number of rows or columns given was negative.
     */
    @NotNull
    public static MatrixSimple onesMatrix(int rows, int cols) {
        ExceptionChecker.assertTrue(rows > 0, new RuntimeException("Can't have 0 or negative amount of rows."));
        ExceptionChecker.assertTrue(cols > 0, new RuntimeException("Can't have 0 or negative amount of columns."));

        double[][] onesVals = new double[rows][cols];
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                onesVals[row][col] = 1;
            }
        }

        return new MatrixSimple(onesVals);
    }

    public static class LUDecomposition {
        public final MatrixSimple P, L, U;
        protected LUDecomposition(MatrixSimple P, MatrixSimple L, MatrixSimple U) {
            this.P = P;
            this.L = L;
            this.U = U;
        }
    }

    public static class QRDecomposition {
        public final MatrixSimple Q, R;
        protected QRDecomposition(MatrixSimple Q, MatrixSimple R) {
            this.Q = Q;
            this.R = R;
        }
    }

    public static class EigenData {
        public final double[] eigenvalues;
        public final MatrixSimple[] eigenvectors;
        public final MatrixSimple eigenbasis;

        private final String eigenvectorString;
        protected EigenData(@NotNull MatrixSimple diagEigenvalueMatrix, @NotNull MatrixSimple eigenVectorMatrix) {
            ExceptionChecker.assertTrue(diagEigenvalueMatrix.isDiagonal(), new ArithmeticException("Matrix not diagonal"));
            ExceptionChecker.assertTrue(eigenVectorMatrix.isSquare(), new ArithmeticException("Matrix not square"));

            this.eigenbasis = eigenVectorMatrix;

            this.eigenvalues = new double[diagEigenvalueMatrix.getNumRows()];
            for (int diag = 0; diag < diagEigenvalueMatrix.getNumRows(); diag++) {
                eigenvalues[diag] = diagEigenvalueMatrix.get(diag, diag);
            }

            eigenvectorString = eigenVectorMatrix.toString();
            eigenvectors = new MatrixSimple[eigenVectorMatrix.getNumCols()];
            for (int col = 0; col < eigenVectorMatrix.getNumCols(); col++) {
                MatrixSimple vector = eigenVectorMatrix.crop(0, eigenVectorMatrix.getNumRows(), col, col+1);
                eigenvectors[col] = vector;
            }
        }

        @Override
        public String toString() {
            return "Eigenvalues "+Arrays.toString(eigenvalues)+"\nEigenvectors:\n"+eigenvectorString;
        }
    }

    public static class Diagonalization {
        public final MatrixSimple X, lambda;
        private final MatrixSimple Xinv;
        private Diagonalization(@NotNull MatrixSimple X, @NotNull MatrixSimple lambda) {
            ExceptionChecker.assertTrue(lambda.isDiagonal(), new ArithmeticException("Matrix must be diagonal"));

            this.X = X;
            this.lambda = lambda;
            this.Xinv = X.invert();
        }

        public MatrixSimple fastPow(int exponent) {
            ExceptionChecker.assertTrue(exponent > 0, new ArithmeticException("matrix exponent must be an integer"));

            MatrixSimple lambdaPow = lambda.copy();
            for (int diag = 0; diag < lambdaPow.getNumRows(); diag++) {
                lambdaPow.set(diag, diag, pow(lambda.get(diag, diag), exponent));
            }

            return X.multiply(lambdaPow).multiply(Xinv);
        }
    }

    @Override
    public String toString() {
        StringBuilder output = new StringBuilder();
        for (double[] row : vals) {
            output.append(Arrays.toString(row));
            output.append('\n');
        }
        output.deleteCharAt(output.length() - 1);
        return output.toString();
    }

    @Override
    public int hashCode() {
        int hashCode = 17;
        for (int row = 0; row < this.getNumRows(); row++) {
            for (int col = 0; col < this.getNumCols(); col++) {
                long bits = Double.doubleToLongBits(this.get(row, col));
                int hash = (int) (bits ^ (bits >>> 32));
                hashCode = 37 * hashCode + hash;
            }
        }
        return hashCode;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof MatrixSimple) {
            MatrixSimple matrix = (MatrixSimple) obj;
            if (this.getNumRows() != matrix.getNumRows() || this.getNumCols() != matrix.getNumCols()) {
                return false;
            }

            boolean equal = true;
            for (int row = 0; row < this.getNumRows(); row++) {
                for (int col = 0; col < this.getNumCols(); col++) {
                    equal &= matrix.get(row, col) == this.get(row, col);
                }
            }
            return equal;
        }
        return false;
    }
}
