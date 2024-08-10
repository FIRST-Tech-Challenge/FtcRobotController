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

import android.opengl.Matrix;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.NonConst;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * An {@link OpenGLMatrix} is a 4x4 matrix commonly used as a transformation matrix for 3D
 * homogeneous coordinates. The data layout of an {@link OpenGLMatrix} is used heavily in the
 * <a href="https://www.opengl.org/">OpenGL</a> high performance graphics standard.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Homogeneous_coordinates">Homogenous coordinates</a>
 * @see <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
 * @see android.opengl.Matrix
 * @see Matrix
 */
public class OpenGLMatrix extends ColumnMajorMatrixF
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    float[] data;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public OpenGLMatrix()
    {
        super(4,4);
        this.data = new float[4*4];
        Matrix.setIdentityM(this.data, 0);
    }

    public OpenGLMatrix(float[] data)
    {
        super(4,4);
        this.data = data;
        if (this.data.length != 4*4) throw dimensionsError();
    }

    /**
     * Constructs an OpenGL matrix whose values are initialized from the other matrix.
     * The other matrix must have dimensions at most 4x4.
     * @param him the matrix from which to initialize our own data
     */
    public OpenGLMatrix(MatrixF him)
    {
        this();
        if (him.numRows > 4 || him.numCols > 4) throw him.dimensionsError();
        for (int i = 0; i < Math.min(4,him.numRows); i++)
        {
            for (int j = 0; j < Math.min(4,him.numCols); j++)
            {
                this.put(i,j, him.get(i,j));
            }
        }
    }

    @Override public MatrixF emptyMatrix(int numRows, int numCols)
    {
        if (numRows==4 && numCols==4)
            return new OpenGLMatrix();
        else
            return new GeneralMatrixF(numRows, numCols);
    }

    /**
     * Creates a matrix for rotation by the indicated angle around the indicated vector.
     */
    public static OpenGLMatrix rotation(AngleUnit angleUnit, float angle, float dx, float dy, float dz)
    {
        float[] data = new float[16];
        Matrix.setRotateM(data, 0, angleUnit.toDegrees(angle), dx, dy, dz);
        return new OpenGLMatrix(data);
    }

    /**
     * Creates a matrix for a rotation specified by three successive rotation angles.
     * @see Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)
     */
    public static OpenGLMatrix rotation(AxesReference axesReference, AxesOrder axesOrder, AngleUnit angleUnit, float first, float second, float third)
    {
        OpenGLMatrix rotation = Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, first, second, third);
        return identityMatrix().multiplied(rotation);
    }
    public static OpenGLMatrix translation(float dx, float dy, float dz)
    {
        OpenGLMatrix result = new OpenGLMatrix();
        result.translate(dx, dy, dz);
        return result;
    }
    public static OpenGLMatrix identityMatrix()
    {
        return new OpenGLMatrix();
    }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    @Override public float[] getData()
    {
        return this.data;
    }

    //----------------------------------------------------------------------------------------------
    // Transformation matrix operations (in-place). These methods all return the receiver
    // in order to facilitate chaining.
    //
    // Note that these are some of the very view matrix operations that update-in-place rather than
    // returning a new matrix and leaving the receiver unmodified. Care must thus be taken to avoid
    // sharing the data of this matrix (using getData()) with other matrix-related objects and then
    // subsequently modifying this matrix.
    //----------------------------------------------------------------------------------------------

    @NonConst public void scale(float scaleX, float scaleY, float scaleZ)
    {
        Matrix.scaleM(this.data, 0, scaleX, scaleY, scaleZ);
    }
    @NonConst public void scale(float scale)
    {
        this.scale(scale, scale, scale);
    }
    @NonConst public void translate(float dx, float dy, float dz)
    {
        Matrix.translateM(this.data, 0, dx, dy, dz);
    }
    @NonConst public void rotate(AngleUnit angleUnit, float angle, float dx, float dy, float dz)
    {
        Matrix.rotateM(this.data, 0, angleUnit.toDegrees(angle), dx, dy, dz);
    }
    @NonConst public void rotate(AxesReference axesReference, AxesOrder axesOrder, AngleUnit angleUnit, float first, float second, float third)
    {
        OpenGLMatrix rotation = Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, first, second, third);
        this.data = this.multiplied(rotation).getData();
    }

    //----------------------------------------------------------------------------------------------
    // Transformation matrix operations
    //----------------------------------------------------------------------------------------------

    @Const public OpenGLMatrix scaled(float scaleX, float scaleY, float scaleZ)
    {
        OpenGLMatrix result = new OpenGLMatrix();
        Matrix.scaleM(result.data, 0, this.data, 0, scaleX, scaleY, scaleZ);
        return result;
    }
    @Const public OpenGLMatrix scaled(float scale)
    {
        return scaled(scale, scale, scale);
    }
    @Const public OpenGLMatrix translated(float dx, float dy, float dz)
    {
        OpenGLMatrix result = new OpenGLMatrix();
        Matrix.translateM(result.data, 0, this.data, 0, dx, dy, dz);
        return result;
    }
    @Const public OpenGLMatrix rotated(AngleUnit angleUnit, float angle, float dx, float dy, float dz)
    {
        OpenGLMatrix result = new OpenGLMatrix();
        Matrix.rotateM(result.data, 0, this.data, 0, angleUnit.toDegrees(angle), dx, dy, dz);
        return result;
    }
    @Const public OpenGLMatrix rotated(AxesReference axesReference, AxesOrder axesOrder, AngleUnit angleUnit, float first, float second, float third)
    {
        OpenGLMatrix rotation = Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, first, second, third);
        return this.multiplied(rotation);
    }

    //----------------------------------------------------------------------------------------------
    // Matrix operations
    //----------------------------------------------------------------------------------------------

    @Override @Const public OpenGLMatrix inverted()
    {
        OpenGLMatrix result = new OpenGLMatrix();
        Matrix.invertM(result.data, 0, this.data, 0);
        return result;
    }

    @Override @Const public OpenGLMatrix transposed()
    {
        return (OpenGLMatrix)super.transposed();
    }

    @Const public OpenGLMatrix multiplied(OpenGLMatrix him)
    {
        OpenGLMatrix result = new OpenGLMatrix();
        Matrix.multiplyMM(result.data, 0, this.data, 0, him.getData(), 0);
        return result;
    }

    @Override @Const public MatrixF multiplied(MatrixF him)
    {
        if (him instanceof OpenGLMatrix)
        {
            return this.multiplied((OpenGLMatrix)him);
        }
        else
            return super.multiplied(him);
    }

    /**
     * Updates the receiver to be the product of itself and another matrix.
     * @param him the matrix with which the receiver is to be multiplied.
     */
    @NonConst public void multiply(OpenGLMatrix him)
    {
        this.data = this.multiplied(him).getData();
    }

    /**
     * Updates the receiver to be the product of itself and another matrix.
     * @param him the matrix with which the receiver is to be multiplied.
     */
    @Override @NonConst public void multiply(MatrixF him)
    {
        if (him instanceof OpenGLMatrix)
        {
            this.multiply((OpenGLMatrix)him);
        }
        else
            super.multiply(him);
    }
}
