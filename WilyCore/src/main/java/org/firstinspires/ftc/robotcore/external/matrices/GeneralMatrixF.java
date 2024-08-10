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

/**
 * A {@link GeneralMatrixF} is a concrete matrix implementation that is supported by
 * a backing store consisting of an array of floats. The matrix is stored in row-major order.
 */
public class GeneralMatrixF extends RowMajorMatrixF
{
    float[] data;

    public GeneralMatrixF(int numRows, int numCols)
    {
        super(numRows, numCols);
        this.data = new float[numRows * numCols];
    }

    private GeneralMatrixF(int numRows, int numCols, int flag)
    {
        super(numRows, numCols);
    }

    public GeneralMatrixF(int numRows, int numCols, float[] data)
    {
        super(numRows, numCols);
        if (data.length != numRows * numCols) throw dimensionsError(numRows, numCols);
        this.data = data;
    }

    @Override public float[] getData()
    {
        return this.data;
    }

    @Override public MatrixF emptyMatrix(int numRows, int numCols)
    {
        return new GeneralMatrixF(numRows, numCols);
    }

    public GeneralMatrixF transposed()
    {
        return (GeneralMatrixF)super.transposed();
    }
}
