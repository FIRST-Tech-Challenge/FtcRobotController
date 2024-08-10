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
package org.firstinspires.ftc.robotcore.external.navigation;

/**
 * {@link AxesOrder} indicates the chronological order of axes about which the three rotations
 * of an {@link Orientation} take place. The geometry of three space is such that there are
 * exactly twelve distinct rotational orders.
 *
 * @see Orientation
 * @see AxesReference
 * @see <a href="https://en.wikipedia.org/wiki/Euler_angles">Euler Angles</a>
 */
public enum AxesOrder
{
    XZX(new int[]{0, 2, 0}), XYX(new int[]{0, 1, 0}), YXY(new int[]{1, 0, 1}),
    YZY(new int[]{1, 2, 1}), ZYZ(new int[]{2, 1, 2}), ZXZ(new int[]{2, 0, 2}),
    XZY(new int[]{0, 2, 1}), XYZ(new int[]{0, 1, 2}), YXZ(new int[]{1, 0, 2}),
    YZX(new int[]{1, 2, 0}), ZYX(new int[]{2, 1, 0}), ZXY(new int[]{2, 0, 1});

    private final int[] indices;

    AxesOrder(int[] indices)
    {
        this.indices = indices;
    }

    /**
     * Returns the numerical axes indices associated with this {@link AxesOrder}.
     * @return the numerical axes indices associated with this {@link AxesOrder}.
     */
    public int[] indices()
    {
        return this.indices;
    }

    /**
     * Returns the {@link Axis axes} associated with this {@link AxesOrder}.
     * @return the {@link Axis axes} associated with this {@link AxesOrder}.
     */
    public Axis[] axes()
    {
        Axis[] result = new Axis[3];
        result[0] = Axis.fromIndex(this.indices[0]);
        result[1] = Axis.fromIndex(this.indices[1]);
        result[2] = Axis.fromIndex(this.indices[2]);
        return result;
    }

    /**
     * Returns the {@link AxesOrder} which is the chronological reverse of the receiver.
     * @return the {@link AxesOrder} which is the chronological reverse of the receiver.
     */
    public AxesOrder reverse()
    {
        switch (this)
        {
            default:
            case XZX: return XZX;
            case XYX: return XYX;
            case YXY: return YXY;
            case YZY: return YZY;
            case ZYZ: return ZYZ;
            case ZXZ: return ZXZ;
            case XZY: return YZX;
            case XYZ: return ZYX;
            case YXZ: return ZXY;
            case YZX: return XZY;
            case ZYX: return XYZ;
            case ZXY: return YXZ;
        }
    }
}
