package com.SCHSRobotics.HAL9001.util.math.geometry;

import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;

import org.jetbrains.annotations.NotNull;

public class VectorSpace {
    private final MatrixSimple basisMatrix;
    public VectorSpace(MatrixSimple @NotNull [] vectors) {
        ExceptionChecker.assertTrue(vectors.length > 0, new ArithmeticException("A basis must have at least one vector."));

        MatrixSimple basisMatrix = vectors[0];
        for (int i = 1; i < vectors.length; i++) {
            basisMatrix = basisMatrix.augment(vectors[i]);
        }
        this.basisMatrix = basisMatrix;
        ExceptionChecker.assertTrue(basisMatrix.nullity() == 0, new ArithmeticException("Basis vectors are not linearly independent"));
    }

    public VectorSpace(@NotNull MatrixSimple basisMatrix) {
        this.basisMatrix = basisMatrix;
        ExceptionChecker.assertTrue(basisMatrix.nullity() == 0, new ArithmeticException("Basis vectors are not linearly independent"));
    }

    public MatrixSimple getBasisMatrix() {
        return basisMatrix;
    }

    public MatrixSimple[] getBasisVectors() {
        return basisMatrix.getCols();
    }

    boolean contains(@NotNull MatrixSimple vector) {
        ExceptionChecker.assertTrue(vector.isVector(), new ArithmeticException("matrix is not a vector."));

        MatrixSimple sol = basisMatrix.augment(vector).rref();
        for (int row = 0; row < sol.getNumRows(); row++) {
            for (int col = 0; col < sol.getNumCols(); col++) {
                if(sol.get(row, col) == 1 && col < basisMatrix.getNumCols()) {
                    break;
                }
                else if(sol.get(row, col) != 0) {
                    return false;
                }
            }
        }
        return true;
    }

    @Override
    public String toString() {
        return basisMatrix.toString();
    }
}
