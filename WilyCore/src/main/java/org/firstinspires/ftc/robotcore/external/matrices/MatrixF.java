/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.external.matrices;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.NonConst;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

/**
 * {@link MatrixF} represents a matrix of floats of a defined dimensionality but abstracts
 * the means by which a particular element of the matrix is retrieved or updated. {@link MatrixF}
 * is an abstract class: it is never instantiated; rather, only instances of its subclasses are
 * made.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Matrix_(mathematics)">Matrix (mathematics)</a>
 * @see <a href="https://en.wikipedia.org/wiki/Matrix_multiplication">Matrix multiplication</a>
 * @see OpenGLMatrix
 * @see GeneralMatrixF
 * @see SliceMatrixF
 */
public abstract class MatrixF
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected int numRows;
    protected int numCols;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Creates a matrix containing the indicated number of rows and columns.
     */
    public MatrixF(int numRows, int numCols)
    {
        this.numRows = numRows;
        this.numCols = numCols;
        if (numRows <= 0 || numCols <= 0) throw dimensionsError();
    }

    /**
     * Returns a matrix which a submatrix of the receiver.
     * @param row       the row in the receiver at which the submatrix is to start
     * @param col       the column in the receiver at which the submatrix is to start
     * @param numRows   the number of rows in the submatrix
     * @param numCols   the number of columns in the submatrix
     * @return the newly created submatrix
     * @see #slice(int, int)
     */
    @Const public SliceMatrixF slice(int row, int col, int numRows, int numCols)
    {
        return new SliceMatrixF(this, row, col, numRows, numCols);
    }

    /**
     * Returns a matrix which is a submatrix of the receiver starting at (0,0)
     * @param numRows   the number of rows in the submatrix
     * @param numCols   the number of columns in the submatrix
     * @return the newly created submatrix
     * @see #slice(int, int, int, int)
     */
    @Const public SliceMatrixF slice(int numRows, int numCols)
    {
        return slice(0,0, numRows, numCols);
    }

    /**
     * Returns an identity matrix of the indicated dimension. An identity matrix is zero
     * everywhere except on the diagonal, where it is one.
     * @param dim the size of the indentity matrix to return
     * @return the new identity matrix
     */
    public static MatrixF identityMatrix(int dim)
    {
        return diagonalMatrix(dim, 1f);
    }

    /**
     * Returns a new matrix which is zero everywhere except on the diagonal, where it has
     * an indicated value.
     * @param dim the size of the matrix to return
     * @param scale the value to place on its diagonal
     * @return the new matrix
     */
    public static MatrixF diagonalMatrix(int dim, float scale)
    {
        GeneralMatrixF result = new GeneralMatrixF(dim, dim);
        for (int i = 0; i < dim; i++)
        {
            result.put(i,i, scale);
        }
        return result;
    }

    /**
     * Returns a new matrix which is zero everywhere, except on the diagonal, where its
     * values are taken from an indicated vector
     * @param vector the values to place on the diagonal
     * @return the new matrix
     */
    public static MatrixF diagonalMatrix(VectorF vector)
    {
        int dim = vector.length();
        GeneralMatrixF result = new GeneralMatrixF(dim, dim);
        for (int i = 0; i < dim; i++)
        {
            result.put(i,i, vector.get(i));
        }
        return result;
    }

    /**
     * Returns a new empty matrix of the indicated dimensions. If a specific implementation
     * associated with the receiver can be used with these dimensions, then such is used; otherwise
     * a general matrix implementation will be used.
     * @return a new empty matrix of the indicated dimensions
     * @see OpenGLMatrix#emptyMatrix(int, int)
     */
    @Const public abstract MatrixF emptyMatrix(int numRows, int numCols);

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    /**
     * Returns the number of rows in this matrix
     * @return the number of rows in this matrix
     */
    @Const public int numRows() { return this.numRows; }

    /**
     * Returns the number of columns in this matrix
     * @return the number of columns in this matrix
     */
    @Const public int numCols() { return this.numCols; }

    /**
     * Returns a particular element of this matrix
     * @param row the index of the row of the element to return
     * @param col the index of the column of the element to return
     * @return the element at the indicated row and column
     * @see #put(int, int, float)
     */
    @Const public abstract float get(int row, int col);

    /**
     * Updates a particular element of this matrix
     * @param row the index of the row of the element to update
     * @param col the index of the column of the element to update
     * @param value the new value for the indicated element
     */
    @NonConst public abstract void put(int row, int col, float value);

    /**
     * Returns a vector containing data of a particular row of the receiver.
     * @param row the row to extract
     * @return a vector containing the data of the indicated row
     */
    @Const public VectorF getRow(int row)
    {
        VectorF result = VectorF.length(this.numCols);
        for (int j = 0; j < numCols; j++)
        {
            result.put(j, this.get(row, j));
        }
        return result;
    }

    /**
     * Returns a vector containing data of a particular column of the receiver.
     * @param col the column to extract
     * @return a vector containing data of the indicated column
     */
    @Const public VectorF getColumn(int col)
    {
        VectorF result = VectorF.length(this.numRows);
        for (int i = 0; i < numRows; i++)
        {
            result.put(i, this.get(i, col));
        }
        return result;
    }

    @Const @Override public String toString()
    {
        StringBuilder result = new StringBuilder();
        result.append("{");
        for (int i = 0; i < this.numRows; i++)
        {
            if (i > 0) result.append(",");
            result.append("{");
            for (int j = 0; j < this.numCols; j++)
            {
                if (j > 0) result.append(",");
                result.append(String.format("%.3f", this.get(i,j)));
            }
            result.append("}");
        }
        result.append("}");
        return result.toString();
    }

    //----------------------------------------------------------------------------------------------
    // Transformation matrix operations
    //----------------------------------------------------------------------------------------------

    /**
     * Transforms the vector according to this matrix interpreted as a transformation matrix.
     * Conversion to <a href="https://en.wikipedia.org/wiki/Homogeneous_coordinates">homogeneous
     * coordinates</a> is automatically provided.
     * @param him the 3D coordinate or 3D homogeneous coordinate that is to be transformed
     * @return the normalized homogeneous coordinate resulting from the transformation.
     *
     * @see <a href="https://en.wikipedia.org/wiki/Homogeneous_coordinates">Homogeneous coordinates</a>
     * @see <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
     * @see VectorF#normalized3D()
     */
    @Const public VectorF transform(VectorF him)
    {
        him = adaptHomogeneous(him);
        return this.multiplied(him).normalized3D();
    }

    /**
     * Automatically adapts vectors to and from homogeneous coordinates according to the
     * size of the receiver matrix.
     * @see #transform(VectorF)
     * @see <a href="https://en.wikipedia.org/wiki/Homogeneous_coordinates">Homogeneous coordinates</a>
     */
    @Const protected VectorF adaptHomogeneous(VectorF him)
    {
        if (this.numCols == 4)
        {
            if (him.length() == 3)
            {
                float[] newData = Arrays.copyOf(him.getData(), 4);
                newData[3] = 1f;
                return new VectorF(newData);
            }
        }
        else if (this.numCols == 3)
        {
            if (him.length() == 4)
            {
                return new VectorF(Arrays.copyOf(him.normalized3D().getData(),3));
            }
        }
        return him;
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being. This should only be invoked on
     * a matrix which is a transformation matrix.
     *
     * We report here using an extrinsic angle reference, meaning that all three angles are
     * rotations in the (fixed) field coordinate system, as this is perhaps easiest to
     * conceptually understand. And we use an angle order of XYZ, which results in the Z
     * angle, being applied last (after X and Y rotations) and so representing the robot's
     * heading on the field, which is often what is of most interest in robot navigation.
     *
     * @return a description of the angles represented by this transformation matrix.
     * @see #formatAsTransform(AxesReference, AxesOrder, AngleUnit)
     * @see <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
     */
    public String formatAsTransform()
    {
        return formatAsTransform(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being. This should only be invoked on
     * a matrix which is a transformation matrix.
     *
     * @param axesReference the reference frame of the angles to use in reporting the transformation
     * @param axesOrder     the order of the angles to use in reporting the transformation
     * @param unit          the angular unit to use in reporting the transformation
     * @return a description of the angles represented by this transformation matrix.
     * @see #formatAsTransform()
     * @see <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
     */
    public String formatAsTransform(AxesReference axesReference, AxesOrder axesOrder, AngleUnit unit)
    {
        /**
         * An easy way to understand what a transform does is to look at the location
         * to which it transforms the origin of the coordinate system. Calling getTranslation()
         * carries out an equivalent computation as it extracts the translational aspect.
         */
        VectorF translation = this.getTranslation();

        /**
         * Figure out in which direction we'd be looking after the transformation. Note that
         * the decomposition of a transformation into orientation angles can be subtle. See
         * {@link Orientation} for a full discussion.
         */
        Orientation orientation = Orientation.getOrientation(this, axesReference, axesOrder, unit);

        return String.format("%s %s", orientation.toString(), translation.toString());
    }

    //----------------------------------------------------------------------------------------------
    // Matrix operations
    //----------------------------------------------------------------------------------------------

    /**
     * Returns a matrix which is the transposition of the receiver matrix.
     * @return a matrix which is the transposition of the receiver matrix.
     */
    @Const public MatrixF transposed()
    {
        MatrixF result = this.emptyMatrix(this.numCols, this.numRows);
        for (int i = 0; i < result.numRows; i++)
        {
            for (int j = 0; j < result.numCols; j++)
            {
                result.put(i,j, this.get(j,i));
            }
        }
        return result;
    }

    /**
     * Updates the receiver to be the product of itself and another matrix.
     * @param him the matrix with which the receiver is to be multiplied.
     */
    @NonConst public void multiply(MatrixF him)
    {
        /**
         * If we multiply C = A x B, the dimensions work out as C(i x k) = A(i x j) B(j x k).
         * If A and C are the same matrix, we have j==k; that is, B must be square.
         */
        if (this.numCols == him.numRows)
        {
            if (him.numRows == him.numCols)
            {
                MatrixF temp = this.multiplied(him);

                // Copy the matrix back
                for (int i = 0; i < this.numRows; i++)
                {
                    for (int j = 0; j < this.numCols; j++)
                    {
                        this.put(i,j, temp.get(i,j));
                    }
                }
            }
            else
                throw dimensionsError();
        }
        else
            throw dimensionsError();
    }

    /**
     * Returns a matrix which is the multiplication of the recevier with another matrix.
     * @param him the matrix with which the receiver is to be multiplied.
     * @return a matrix which is the product of the two matrices
     */
    @Const public MatrixF multiplied(MatrixF him)
    {
        if (this.numCols == him.numRows)
        {
            MatrixF result = this.emptyMatrix(this.numRows, him.numCols);
            for (int i = 0; i < result.numRows; i++)
            {
                for (int j = 0; j < result.numCols; j++)
                {
                    float sum = 0f;
                    for (int k = 0; k < this.numCols; k++)
                    {
                        sum += this.get(i, k) * him.get(k, j);
                    }
                    result.put(i,j,sum);
                }
            }
            return result;
        }
        else
            throw dimensionsError();
    }

    /**
     * Returns a new matrix in which all the entries of the receiver have been scaled
     * by an indicated value.
     * @param scale the factor with which to scale each entry of the receiver
     * @return the new, scaled matrix
     */
    @Const public MatrixF multiplied(float scale)
    {
        MatrixF result = this.emptyMatrix(this.numCols, this.numRows);
        for (int i = 0; i < result.numRows; i++)
        {
            for (int j = 0; j < result.numCols; j++)
            {
                result.put(i,j, this.get(i,j) * scale);
            }
        }
        return result;
    }

    @NonConst public void multiply(float scale)
    {
        for (int i = 0; i < this.numRows; i++)
        {
            for (int j = 0; j < this.numCols; j++)
            {
                this.put(i,j, this.get(i,j) * scale);
            }
        }
    }

    /**
     * Multiplies the receiver by the indicated vector, considered as a column matrix.
     * @param him the vector with which the receiver is to be multiplied
     * @return a matrix which is the product of the receiver and the vector
     */
    @Const public VectorF multiplied(VectorF him)
    {
        return this.multiplied(new ColumnMatrixF(him)).toVector();
    }

    @NonConst public void multiply(VectorF him)
    {
        VectorF result = this.multiplied(new ColumnMatrixF(him)).toVector();
        for (int i = 0; i < result.length(); i++)
        {
            this.put(i,0, result.get(i));
        }
    }

    /**
     * Multiplies the receiver by the indicated vector, considered as a column matrix.
     * @param him the vector with which the receiver is to be multiplied
     * @return a matrix which is the product of the receiver and the vector
     */
    @Const public VectorF multiplied(float[] him)
    {
        return this.multiplied(new VectorF(him));
    }

    @NonConst public void multiply(float[] him)
    {
        VectorF result = this.multiplied(new VectorF(him));
        for (int i = 0; i < result.length(); i++)
        {
            this.put(i,0, result.get(i));
        }
    }

    /**
     * If the receiver is one-dimensional in one of its dimensions, returns a vector
     * containing the data of the receiver; otherwise, an exception is thrown.
     * @return a vector containing the data of the receiver
     */
    @Const public VectorF toVector()
    {
        if (this.numCols == 1)
        {
            VectorF result = VectorF.length(this.numRows);
            for (int i = 0; i < this.numRows; i++)
            {
                result.put(i, this.get(i,0));
            }
            return result;
        }
        else if (this.numRows == 1)
        {
            VectorF result = VectorF.length(this.numCols);
            for (int j = 0; j < this.numCols; j++)
            {
                result.put(j, this.get(0,j));
            }
            return result;
        }
        else
            throw dimensionsError();
    }

    /**
     * Returns a new matrix whose elements are the sum of the corresponding elements of
     * the receiver and the addend
     * @param addend the matrix which is to be added to the receiver
     * @return the new matrix
     */
    @Const public MatrixF added(MatrixF addend)
    {
        if (this.numRows==addend.numRows && this.numCols==addend.numCols)
        {
            MatrixF result = this.emptyMatrix(this.numRows, this.numCols);
            for (int i = 0; i < result.numRows; i++)
            {
                for (int j = 0; j < result.numCols; j++)
                {
                    result.put(i,j, this.get(i,j) + addend.get(i,j));
                }
            }
            return result;
        }
        else
            throw  dimensionsError();
    }

    /**
     * Adds a matrix, in place, to the receiver
     * @param addend the matrix which is to be added to the receiver
     */
    @NonConst public void add(MatrixF addend)
    {
        if (this.numRows==addend.numRows && this.numCols==addend.numCols)
        {
            for (int i = 0; i < this.numRows; i++)
            {
                for (int j = 0; j < this.numCols; j++)
                {
                    this.put(i,j, this.get(i,j) + addend.get(i,j));
                }
            }
        }
        else
            throw  dimensionsError();
    }

    /**
     * Returns a new matrix whose elements are the difference of the corresponding elements of
     * the receiver and the subtrahend
     * @param subtrahend the matrix which is to be subtracted from the receiver
     * @return the new matrix
     */
    @Const public MatrixF subtracted(MatrixF subtrahend)
    {
        if (this.numRows==subtrahend.numRows && this.numCols==subtrahend.numCols)
        {
            MatrixF result = this.emptyMatrix(this.numRows, this.numCols);
            for (int i = 0; i < result.numRows; i++)
            {
                for (int j = 0; j < result.numCols; j++)
                {
                    result.put(i,j, this.get(i,j) - subtrahend.get(i,j));
                }
            }
            return result;
        }
        else
            throw  dimensionsError();
    }

    /**
     * Subtracts a matrix, in place, from the receiver.
     * @param subtrahend the matrix which is to be subtracted from the receiver
     */
    @NonConst public void subtract(MatrixF subtrahend)
    {
        if (this.numRows==subtrahend.numRows && this.numCols==subtrahend.numCols)
        {
            for (int i = 0; i < this.numRows; i++)
            {
                for (int j = 0; j < this.numCols; j++)
                {
                    this.put(i,j, this.get(i,j) - subtrahend.get(i,j));
                }
            }
        }
        else
            throw  dimensionsError();
    }

    /** @see #added(MatrixF) */
    @Const public MatrixF added(VectorF him)
    {
        return this.added(new ColumnMatrixF(him));
    }
    /** @see #added(VectorF) */
    @Const public MatrixF added(float[] him)
    {
        return this.added(new VectorF(him));
    }
    /** @see #subtracted(MatrixF) */
    @Const public MatrixF subtracted(VectorF him)
    {
        return this.subtracted(new ColumnMatrixF(him));
    }
    /** @see #subtracted(VectorF) */
    @Const public MatrixF subtracted(float[] him)
    {
        return this.subtracted(new VectorF(him));
    }

    /** @see #add(MatrixF) */
    @NonConst public void add(VectorF him)
    {
        this.add(new ColumnMatrixF(him));
    }
    /** @see #add(VectorF) */
    @NonConst public void add(float[] him)
    {
        this.add(new VectorF(him));
    }
    /** @see #subtract(MatrixF) */
    @NonConst public void subtract(VectorF him)
    {
        this.subtract(new ColumnMatrixF(him));
    }
    /** @see #subtract(VectorF) */
    @NonConst public void subtract(float[] him)
    {
        this.subtract(new VectorF(him));
    }

    //----------------------------------------------------------------------------------------------
    // Transformations
    //----------------------------------------------------------------------------------------------

    /**
     * Assumes that the receiver is non-perspective transformation matrix. Returns the translation
     * component of the transformation.
     * @return the translation component of the transformation
     */
    @Const public VectorF getTranslation()
    {
        return this.getColumn(3).normalized3D();
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    protected RuntimeException dimensionsError()
    {
        return dimensionsError(this.numRows, this.numCols);
    }

    @SuppressLint("DefaultLocale")
    protected static RuntimeException dimensionsError(int numRows, int numCols)
    {
        return new IllegalArgumentException(String.format("matrix dimensions are incorrect: rows=%d cols=%d", numRows, numCols));
    }

    //----------------------------------------------------------------------------------------------
    // Inverses (at end because of verbosity)
    //----------------------------------------------------------------------------------------------

    /**
     * Returns a matrix which is the matrix-multiplication inverse of the receiver.
     * @return a matrix which is the matrix-multiplication inverse of the receiver
     */
    @Const public MatrixF inverted()
    {
        // Algorithms were generated with the help of Mathematica: general nxn matrices with symbolic
        // (instead of numeric) entries were defined, their inverse symbolically computed, then
        // automatically transcribed to Java.

        if (this.numRows != this.numCols) throw dimensionsError();

        if (this.numRows == 4)
        {
            MatrixF result = this.emptyMatrix(4,4);

            final float m00=get(0,0), m01=get(0,1), m02=get(0,2), m03=get(0,3);
            final float m10=get(1,0), m11=get(1,1), m12=get(1,2), m13=get(1,3);
            final float m20=get(2,0), m21=get(2,1), m22=get(2,2), m23=get(2,3);
            final float m30=get(3,0), m31=get(3,1), m32=get(3,2), m33=get(3,3);

            final float denom = m00 * m11 * m22 * m33
                    + m00 * m12 * m23 * m31
                    + m00 * m13 * m21 * m32
                    + m01 * m10 * m23 * m32
                    + m01 * m12 * m20 * m33
                    + m01 * m13 * m22 * m30
                    + m02 * m10 * m21 * m33
                    + m02 * m11 * m23 * m30
                    + m02 * m13 * m20 * m31
                    + m03 * m10 * m22 * m31
                    + m03 * m11 * m20 * m32
                    + m03 * m12 * m21 * m30
                    - m01 * m10 * m22 * m33
                    - m00 * m12 * m21 * m33
                    - m02 * m11 * m20 * m33
                    - m00 * m11 * m23 * m32
                    - m03 * m10 * m21 * m32
                    - m01 * m13 * m20 * m32
                    - m02 * m10 * m23 * m31
                    - m00 * m13 * m22 * m31
                    - m03 * m12 * m20 * m31
                    - m01 * m12 * m23 * m30
                    - m03 * m11 * m22 * m30
                    - m02 * m13 * m21 * m30;

            result.put(0, 0, (m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32 - m12 * m21 * m33 - m11 * m23 * m32 - m13 * m22 * m31) / denom);
            result.put(0, 1, (m01 * m23 * m32 + m02 * m21 * m33 + m03 * m22 * m31 - m01 * m22 * m33 - m03 * m21 * m32 - m02 * m23 * m31) / denom);
            result.put(0, 2, (m01 * m12 * m33 + m02 * m13 * m31 + m03 * m11 * m32 - m02 * m11 * m33 - m01 * m13 * m32 - m03 * m12 * m31) / denom);
            result.put(0, 3, (m01 * m13 * m22 + m02 * m11 * m23 + m03 * m12 * m21 - m01 * m12 * m23 - m03 * m11 * m22 - m02 * m13 * m21) / denom);
            result.put(1, 0, (m10 * m23 * m32 + m12 * m20 * m33 + m13 * m22 * m30 - m10 * m22 * m33 - m13 * m20 * m32 - m12 * m23 * m30) / denom);
            result.put(1, 1, (m00 * m22 * m33 + m02 * m23 * m30 + m03 * m20 * m32 - m02 * m20 * m33 - m00 * m23 * m32 - m03 * m22 * m30) / denom);
            result.put(1, 2, (m00 * m13 * m32 + m02 * m10 * m33 + m03 * m12 * m30 - m00 * m12 * m33 - m03 * m10 * m32 - m02 * m13 * m30) / denom);
            result.put(1, 3, (m00 * m12 * m23 + m02 * m13 * m20 + m03 * m10 * m22 - m02 * m10 * m23 - m00 * m13 * m22 - m03 * m12 * m20) / denom);
            result.put(2, 0, (m10 * m21 * m33 + m11 * m23 * m30 + m13 * m20 * m31 - m11 * m20 * m33 - m10 * m23 * m31 - m13 * m21 * m30) / denom);
            result.put(2, 1, (m00 * m23 * m31 + m01 * m20 * m33 + m03 * m21 * m30 - m00 * m21 * m33 - m03 * m20 * m31 - m01 * m23 * m30) / denom);
            result.put(2, 2, (m00 * m11 * m33 + m01 * m13 * m30 + m03 * m10 * m31 - m01 * m10 * m33 - m00 * m13 * m31 - m03 * m11 * m30) / denom);
            result.put(2, 3, (m00 * m13 * m21 + m01 * m10 * m23 + m03 * m11 * m20 - m00 * m11 * m23 - m03 * m10 * m21 - m01 * m13 * m20) / denom);
            result.put(3, 0, (m10 * m22 * m31 + m11 * m20 * m32 + m12 * m21 * m30 - m10 * m21 * m32 - m12 * m20 * m31 - m11 * m22 * m30) / denom);
            result.put(3, 1, (m00 * m21 * m32 + m01 * m22 * m30 + m02 * m20 * m31 - m01 * m20 * m32 - m00 * m22 * m31 - m02 * m21 * m30) / denom);
            result.put(3, 2, (m00 * m12 * m31 + m01 * m10 * m32 + m02 * m11 * m30 - m00 * m11 * m32 - m02 * m10 * m31 - m01 * m12 * m30) / denom);
            result.put(3, 3, (m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21 - m01 * m10 * m22 - m00 * m12 * m21 - m02 * m11 * m20) / denom);

            return result;
        }

        if (this.numRows == 3)
        {
            MatrixF result = this.emptyMatrix(3,3);

            final float m00=get(0,0), m01=get(0,1), m02=get(0,2);
            final float m10=get(1,0), m11=get(1,1), m12=get(1,2);
            final float m20=get(2,0), m21=get(2,1), m22=get(2,2);

            final float denom = m00 * m11 * m22
                    + m01 * m12 * m20
                    + m02 * m10 * m21
                    - m01 * m10 * m22
                    - m00 * m12 * m21
                    - m02 * m11 * m20;

            result.put(0, 0, (m11 * m22 - m12 * m21) / denom);
            result.put(0, 1, (m02 * m21 - m01 * m22) / denom);
            result.put(0, 2, (m01 * m12 - m02 * m11) / denom);
            result.put(1, 0, (m12 * m20 - m10 * m22) / denom);
            result.put(1, 1, (m00 * m22 - m02 * m20) / denom);
            result.put(1, 2, (m02 * m10 - m00 * m12) / denom);
            result.put(2, 0, (m10 * m21 - m11 * m20) / denom);
            result.put(2, 1, (m01 * m20 - m00 * m21) / denom);
            result.put(2, 2, (m00 * m11 - m01 * m10) / denom);

            return result;
        }

        if (this.numRows == 2)
        {
            MatrixF result = this.emptyMatrix(2,2);

            final float m00=get(0,0), m01=get(0,1);
            final float m10=get(1,0), m11=get(1,1);

            final float denom = m00 * m11 - m01 * m10;

            result.put(0, 0, (m11) / denom);
            result.put(0, 1, (-m01) / denom);
            result.put(1, 0, (-m10) / denom);
            result.put(1, 1, (m00) / denom);

            return result;
        }

        if (this.numRows == 1)
        {
            MatrixF result = this.emptyMatrix(1,1);
            result.put(0,0, 1 / get(0,0));
            return result;
        }

        throw dimensionsError(); // really NYI: we haven't bothered to code other cases
    }

}