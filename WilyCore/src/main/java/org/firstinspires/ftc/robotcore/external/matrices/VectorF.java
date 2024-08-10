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

/**
 * A {@link VectorF} represents a single-dimensional vector of floats. It is <em>not</em> a matrix,
 * but can easily be converted into either a {@link RowMatrixF} or a {@link ColumnMatrixF} should
 * that be desired. That said, vectors can be multiplied by matrices to their left (or right); this
 * is commonly used to transform a set of coordinates (in the vector) by a transformation matrix.
 *
 * @see MatrixF
 * @see RowMatrixF
 * @see ColumnMatrixF
 */
public class VectorF
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected float[] data;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Creates a new vector of the indicated length. The vector will contain zeros.
     * @param length the length of the new vector to return
     * @return the newly created vector
     */
    public static VectorF length(int length)
    {
        return new VectorF(new float[length]);
    }

    public VectorF(float[] data)
    {
        this.data = data;
    }

    public VectorF(float x)
    {
        this.data = new float[1];
        this.data[0] = x;
    }

    public VectorF(float x, float y)
    {
        this.data = new float[2];
        this.data[0] = x;
        this.data[1] = y;
    }

    public VectorF(float x, float y, float z)
    {
        this.data = new float[3];
        this.data[0] = x;
        this.data[1] = y;
        this.data[2] = z;
    }

    public VectorF(float x, float y, float z, float w)
    {
        this.data = new float[4];
        this.data[0] = x;
        this.data[1] = y;
        this.data[2] = z;
        this.data[3] = w;
    }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    @Const public float[] getData()
    {
        return this.data;
    }

    @Const public int length()
    {
        return this.data.length;
    }

    @Const public float get(int index)
    {
        return this.data[index];
    }

    @NonConst public void put(int index, float value)
    {
        this.data[index] = value;
    }

    @Override public String toString()
    {
        StringBuilder result = new StringBuilder();
        result.append("{");
        for (int i = 0; i < this.length(); i++)
        {
            if (i > 0) result.append(" ");
            result.append(String.format("%.2f", this.data[i]));
        }
        result.append("}");
        return result.toString();
    }

    //----------------------------------------------------------------------------------------------
    // Transformation matrix operations
    //----------------------------------------------------------------------------------------------

    /**
     * Consider this vector as a 3D coordinate or 3D homogeneous coordinate, and, if the
     * latter, return its normalized form. In either case, the result is of length three, and
     * contains coordinate values for x, y, and z at indices 0, 1, and 2 respectively.
     * @return the normalized form of this coordinate vector
     *
     * @see <a href="https://en.wikipedia.org/wiki/Homogeneous_coordinates">Homogeneous coordinates</a>
     */
    @Const public VectorF normalized3D()
    {
        if (this.length()==3)
        {
            return this;
        }
        else if (this.length()==4)
        {
            return new VectorF(
                    this.data[0]/this.data[3],
                    this.data[1]/this.data[3],
                    this.data[2]/this.data[3]);
        }
        else
            throw dimensionsError();
    }

    //----------------------------------------------------------------------------------------------
    // Matrix Operations
    //----------------------------------------------------------------------------------------------

    @Const public float magnitude()
    {
        return (float)Math.sqrt(this.dotProduct(this));
    }

    /**
     * Returns the dot product of this vector and another.
     * @param him the other vector with whom the dot product is to be formed
     * @return the dot product of this vector and another.
     *
     * @see <a href="https://en.wikipedia.org/wiki/Dot_product">Dot product</a>
     */
    @Const public float dotProduct(VectorF him)
    {
        if (this.length() == him.length())
        {
            float sum = 0;
            for (int i = 0; i < this.length(); i++)
            {
                sum += this.get(i) * him.get(i);
            }
            return sum;
        }
        else
            throw dimensionsError();
    }

    /**
     * Multiplies this vector, taken as a row vector, against the indicated matrix.
     */
    @Const public MatrixF multiplied(MatrixF him)
    {
        return new RowMatrixF(this).multiplied(him);
    }

    /**
     * Adds this vector, taken as a row vector against, to the indicated matrix.
     */
    @Const public MatrixF added(MatrixF addend)
    {
        return new RowMatrixF(this).added(addend);
    }

    @Const public VectorF added(VectorF addend)
    {
        if (this.length() == addend.length())
        {
            VectorF result = VectorF.length(this.length());
            for (int i = 0; i < this.length(); i++)
            {
                result.put(i, this.get(i) + addend.get(i));
            }
            return result;
        }
        else
            throw dimensionsError();
    }

    @NonConst public void add(VectorF addend)
    {
        if (this.length() == addend.length())
        {
            for (int i = 0; i < this.length(); i++)
            {
                this.put(i, this.get(i) + addend.get(i));
            }
        }
        else
            throw dimensionsError();
    }

    /**
     * Subtracts the indicated matrix from this vector, taken as a row vector.
     */
    @Const public MatrixF subtracted(MatrixF subtrahend)
    {
        return new RowMatrixF(this).subtracted(subtrahend);
    }

    @Const public VectorF subtracted(VectorF subtrahend)
    {
        if (this.length() == subtrahend.length())
        {
            VectorF result = VectorF.length(this.length());
            for (int i = 0; i < this.length(); i++)
            {
                result.put(i, this.get(i) - subtrahend.get(i));
            }
            return result;
        }
        else
            throw dimensionsError();
    }

    @NonConst public void subtract(VectorF subtrahend)
    {
        if (this.length() == subtrahend.length())
        {
            for (int i = 0; i < this.length(); i++)
            {
                this.put(i, this.get(i) - subtrahend.get(i));
            }
        }
        else
            throw dimensionsError();
    }

    /**
     * Returns a new vector containing the elements of this vector scaled by the indicated factor.
     */
    @Const public VectorF multiplied(float scale)
    {
        VectorF result = VectorF.length(this.length());
        for (int i = 0; i < this.length(); i++)
        {
            result.put(i, this.get(i) * scale);
        }
        return result;
    }

    @NonConst public void multiply(float scale)
    {
        for (int i = 0; i < this.length(); i++)
        {
            this.put(i, this.get(i) * scale);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    protected RuntimeException dimensionsError()
    {
        return dimensionsError(this.length());
    }

    @SuppressLint("DefaultLocale") protected static RuntimeException dimensionsError(int length)
    {
        return new IllegalArgumentException(String.format("vector dimensions are incorrect: length=%d", length));
    }
}
