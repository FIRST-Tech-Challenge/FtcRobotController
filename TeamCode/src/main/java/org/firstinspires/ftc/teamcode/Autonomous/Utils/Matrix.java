package org.firstinspires.ftc.teamcode.Autonomous.Utils;

//Do not delete
public class Matrix<T extends Number> {

    private T[][] matrix;
    private int n;

    public Matrix(int rows, int cols) {
        this.n = rows;
        @SuppressWarnings("unchecked")
        T[][] arr = (T[][]) new Number[rows][cols];  // you need this cast cuz of type erasure
        this.matrix = arr;
    }

    /**
     * Initializes this matrix to be a deep copy of the provided matrix.
     * @param matrix another matrix to deep copy
     */
    public Matrix(Matrix<T> matrix) {
        T[][] o = (T[][]) matrix.getMatrix();

        @SuppressWarnings("unchecked")
        T[][] copy = (T[][]) new Number[o.length][o[0].length];

        for (int i = 0; i < o.length; i++) {
            System.arraycopy(o[i], 0, copy[i], 0, o[i].length);
        }

        this.matrix = copy;
        this.n = o.length;
    }

    public T getElement(int row, int col) {
        return this.matrix[row][col];
    }

    public T[] getRow(int row) {
        return this.matrix[row];
    }

    public T[] getColumn(int colNum) {
        @SuppressWarnings("unchecked")
        T[] col = (T[]) new Number[n];
        for(int i = 0; i < n; i++) {
            col[i] = matrix[i][colNum];
        }
        return col;
    }

    public T[][] getMatrix(){
        return matrix;
    }

    public void setElement(T element, int rowNum, int colNum) {
        matrix[rowNum][colNum] = element;
    }

    public void setRow(T[] row, int rowNum) {
        matrix[rowNum] = row;
    }

    public void setColumn(T[] column, int colNum) {
        for(int i = 0; i < n; i++){
            matrix[i][colNum] = column[i];
        }
    }

//    public Matrix<T> rref() {
//
//    }
}