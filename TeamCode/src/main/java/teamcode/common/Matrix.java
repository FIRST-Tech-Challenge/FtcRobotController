package teamcode.common;

public class Matrix implements Cloneable{
    private double[][] matrix;
    private int rows; //m
    private int columns; //n

    public Matrix(int rows, int columns){
        this.rows = rows;
        this.columns = columns;
        matrix = new double[rows][columns];
    }

    public Matrix(double[][] matrix){
        this.matrix = matrix;
        rows = matrix.length;
        columns = matrix[0].length;
    }

    public Matrix(Matrix m){
        this(m.matrix);
    }

    public static Matrix identity(int size) {
        Matrix identity = new Matrix(size, size);
        for (int i = 0; i < size; i++)
            identity.matrix[i][i] = 1;
        return identity;
    }
    public Matrix transpose() {
        Matrix transpose = new Matrix(rows, columns);
        for (int i = 0; i < columns; i++) {
            for (int j = 0; j < rows; j++) {
                transpose.matrix[j][i] = this.matrix[i][j];
            }
        }
        return transpose;
    }

    public Matrix add(Matrix other) {
        if (other.rows != rows || other.columns != columns){
            throw new RuntimeException("Illegal matrix dimensions.");
        }
        Matrix sum = new Matrix(rows, columns);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                sum.matrix[i][j] = matrix[i][j] + other.matrix[i][j];
            }
        }
        return sum;
    }



    public Matrix subtract(Matrix other) {
        if (other.rows != rows || other.columns != columns){
            throw new RuntimeException("Illegal matrix dimensions.");
        }
        Matrix sum = new Matrix(rows, columns);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                sum.matrix[i][j] = matrix[i][j] - other.matrix[i][j];
            }
        }
        return sum;
    }
    // does A = B exactly?
    public boolean eq(Matrix other) {
        if (other.rows != rows || other.columns != columns){
            throw new RuntimeException("Illegal matrix dimensions.");
        }
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                if (matrix[i][j] != other.matrix[i][j]) {
                    return false;
                }
            }
        }
        return true;
    }

    // return C = A * B
    public Matrix multiply(Matrix other) {
        if (columns != other.rows){
            throw new RuntimeException("Illegal matrix dimensions.");
        }
        Matrix product = new Matrix(rows, other.columns);
        for (int i = 0; i < product.rows; i++) {
            for (int j = 0; j < product.columns; j++) {
                for (int k = 0; k < rows; k++) {
                    product.matrix[i][j] += (matrix[i][k] * other.matrix[k][j]);
                }
            }
        }
        return product;
    }

    /**
     * multiplies the matrix by a scalar
     * @param scalar the constant that the matrix is being multiplied by
     */
    public void multiply(double scalar){
        for(int i = 0; i < matrix.length; i++){
            for(int j = 0; j < matrix[0].length; j++){
                matrix[i][j] *= scalar;
            }
        }
    }

    public Matrix inverse(Matrix m){
        return m.inverse();
    }

    public Matrix inverse () {
        if(isSquare() && getDeterminant(this) != 0){
            //calculate the inverse
            double determinant = getDeterminant(this);
            Matrix determinants = new Matrix(rows, columns);
            int counter = 0;
            for(int i = 0; i < rows; i++){
                for(int j = 0; j < columns; j++){
                    double value = getDeterminant(createSubmatrix(i, j));
                    if (counter % 2 == 1) {
                        value *= -1;
                    }
                    determinants.setValue(i, j, value);
                    counter++;
                }
            }
            //System.out.println(determinants);
            double[][] mirroredInverse = new double[determinants.rows][determinants.columns];
            for(int i = 0; i < determinants.rows; i++){
                for(int j = 0; j < determinants.columns; j++){
                    mirroredInverse[i][j] = determinants.matrix[j][i];
                }
            }

            Matrix inverse = new Matrix(mirroredInverse);
            inverse.multiply(1.0 / determinant);
            return inverse;
        }else{
            return pseudoInverse();
        }

    }

    public Matrix createSubmatrix(int excludedRow, int excludedColumn){
        Matrix subMatrix = new Matrix(matrix.length - 1, matrix[0].length - 1);
        int rowIndice = 0;
        for(int i = 0;i < matrix.length; i++){

            int columnIndice = 0;
            for(int j = 0; j < matrix[0].length; j++){
                if(i != excludedRow && j != excludedColumn){
                    subMatrix.matrix[rowIndice][columnIndice] = matrix[i][j];
                    columnIndice++;
                }
            }
            if(i != excludedRow){
                rowIndice++;
            }

        }
        return subMatrix;
    }


    public static double getDeterminant(Matrix matrix) {
        if(matrix.isSquare()){
            if(matrix.rows == 1){
                return matrix.matrix[0][0];
            }else if(matrix.rows == 2){
                return (matrix.matrix[0][0] * matrix.matrix[1][1]) - (matrix.matrix[0][1] * matrix.matrix[1][0]);
            }else{
                Matrix submatrix = new Matrix(matrix.rows - 1, matrix.columns - 1);
                double determinant = 0;
                for(int i = 0; i < matrix.columns; i++){
                    if(i % 2 == 0){
                        determinant += matrix.matrix[0][i] * getDeterminant(createSubmatrix(i, matrix.matrix));
                    }else{
                        determinant -= matrix.matrix[0][i] * getDeterminant(createSubmatrix(i, matrix.matrix));
                    }
                }
                return determinant;
            }
        }
        return 0;
    }

    private static Matrix createSubmatrix(int excludedIndex, double[][] matrixData) {
        Matrix submatrix = new Matrix(matrixData.length - 1, matrixData[0].length - 1);
        boolean isExcluded = false;
        for(int i = 1; i <= submatrix.rows; i++){
            for(int j = 0; j < matrixData.length; j++){
                if(j == excludedIndex){
                    j++;
                    isExcluded = true;
                }
                if(isExcluded) {
                    if(j == matrixData.length){
                        int k = j - 2;
                        submatrix.matrix[i - 1][k] = matrixData[i][k];
                    }else {
                        submatrix.matrix[i - 1][j - 1] = matrixData[i][j];
                    }
                }else{
                    submatrix.matrix[i - 1][j] = matrixData[i][j];
                }
            }
            isExcluded = false;
        }
        return submatrix;
    }

    public void setValue(int row, int column, double value){
        matrix[row][column] = value;
    }

    public double getValue(int row, int column){
        return matrix[row][column];
    }

    private Matrix pseudoInverse() {
        Matrix transposed = this.transpose();
        Matrix inverse = inverse(this.multiply(transposed));
        return inverse.multiply(transposed);
    }


    public boolean  isSquare(){
        return rows == columns;
    }

    public Matrix clone(){
        return new Matrix(this.matrix);
    }

    public double[][] getMatrix(){
        return matrix;
    }

    @Override
    public String toString() {
        String result = "";
        for(int i = 0; i < rows; i++){
            for (int j = 0; j < columns; j++){
                result += matrix[i][j] + " , ";
            }
            result += "\n";
        }
        return result;
    }
}