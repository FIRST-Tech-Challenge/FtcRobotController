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
 * A {@link SliceMatrixF} is a matrix whose implementation is a submatrix of some other matrix.
 */
public class SliceMatrixF extends MatrixF
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected MatrixF matrix;
    protected int     row;
    protected int     col;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Creates a {@link SliceMatrixF} based on the indicated matrix whose upper left corner is at
     * (row, col) of that matrix and whose size is numRows x numCols.
     * @param matrix    the matrix we are to take a slice of
     * @param row       the row in matrix in which the slice is to begin
     * @param col       the column in matrix in which the slice is to begin
     * @param numRows   the number of rows that the slice should be
     * @param numCols   the number of columns that the slice should be
     */
    public SliceMatrixF(MatrixF matrix, int row, int col, int numRows, int numCols)
    {
        super(numRows, numCols);
        this.matrix = matrix;
        this.row = row;
        this.col = col;

        if (row + numRows >= matrix.numRows) throw dimensionsError();
        if (col + numCols >= matrix.numCols) throw dimensionsError();
    }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    @Override public float get(int row, int col)
    {
        return this.matrix.get(this.row + row, this.col + col);
    }

    @Override public void put(int row, int col, float value)
    {
        this.matrix.put(this.row + row, this.col + col, value);
    }

    @Override public MatrixF emptyMatrix(int numRows, int numCols)
    {
        return this.matrix.emptyMatrix(numRows, numCols);
    }
}
