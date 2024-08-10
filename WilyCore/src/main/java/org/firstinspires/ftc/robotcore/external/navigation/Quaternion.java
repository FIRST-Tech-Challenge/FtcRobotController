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

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.Locale;

/**
 * A {@link Quaternion} can indicate an orientation in three-space without the trouble of
 * possible gimbal-lock.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Quaternion">https://en.wikipedia.org/wiki/Quaternion</a>
 * @see <a href="https://en.wikipedia.org/wiki/Gimbal_lock">https://en.wikipedia.org/wiki/Gimbal_lock</a>
 * @see <a href="https://www.youtube.com/watch?v=zc8b2Jo7mno">https://www.youtube.com/watch?v=zc8b2Jo7mno</a>
 * @see <a href="https://www.youtube.com/watch?v=mHVwd8gYLnI">https://www.youtube.com/watch?v=mHVwd8gYLnI</a>
 */
public class Quaternion
{
    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    public static Quaternion identityQuaternion()
    {
        return new Quaternion(1.0f, 0.0f, 0.0f, 0.0f, 0);
    }

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public float w;
    public float x;
    public float y;
    public float z;

    /**
     * the time on the System.nanoTime() clock at which the data was acquired. If no
     * timestamp is associated with this particular set of data, this value is zero.
     */
    public long acquisitionTime;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public Quaternion()
    {
        this(0, 0, 0, 0, 0);
    }

    public Quaternion(float w, float x, float y, float z, long acquisitionTime)
    {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
        this.acquisitionTime = acquisitionTime;
    }

    public static Quaternion fromMatrix(MatrixF m, long acquisitionTime)
    {
        // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        float tr = m.get(0,0) + m.get(1,1) + m.get(2, 2);

        float w, x, y, z;

        if (tr > 0)
        {
            float s = (float) (Math.sqrt(tr + 1.0) * 2); // S=4*w
            w = (float) (0.25 * s);
            x = (m.get(2, 1) - m.get(1, 2)) / s;
            y = (m.get(0, 2) - m.get(2, 0)) / s;
            z = (m.get(1, 0) - m.get(0, 1)) / s;
        }
        else if ((m.get(0, 0) > m.get(1, 1)) & (m.get(0, 0) > m.get(2, 2)))
        {
            float s = (float) (Math.sqrt(1.0 + m.get(0, 0) - m.get(1, 1) - m.get(2, 2)) * 2); // S=4*x
            w = (m.get(2, 1) - m.get(1, 2)) / s;
            x = (float) (0.25 * s);
            y = (m.get(0, 1) + m.get(1, 0)) / s;
            z = (m.get(0, 2) + m.get(2, 0)) / s;
        }
        else if (m.get(1, 1) > m.get(2, 2))
        {
            float s = (float) (Math.sqrt(1.0 + m.get(1, 1) - m.get(0, 0) - m.get(2, 2)) * 2); // S=4*y
            w = (m.get(0, 2) - m.get(2, 0)) / s;
            x = (m.get(0, 1) + m.get(1, 0)) / s;
            y = (float) (0.25 * s);
            z = (m.get(1, 2) + m.get(2, 1)) / s;
        }
        else
        {
            float s = (float) (Math.sqrt(1.0 + m.get(2, 2) - m.get(0, 0) - m.get(1, 1)) * 2); // S=4*z
            w = (m.get(1, 0) - m.get(0, 1)) / s;
            x = (m.get(0, 2) + m.get(2, 0)) / s;
            y = (m.get(1, 2) + m.get(2, 1)) / s;
            z = (float) (0.25 * s);
        }

        return new Quaternion(w, x, y, z, acquisitionTime).normalized();
    }

    //----------------------------------------------------------------------------------------------
    // Operations
    //----------------------------------------------------------------------------------------------

    public float magnitude()
    {
        return (float)Math.sqrt(w*w + x*x + y*y + z*z);
    }

    public Quaternion normalized()
    {
        float mag = this.magnitude();
        return new Quaternion(
                w / mag,
                x / mag,
                y / mag,
                z / mag,
                this.acquisitionTime);
    }

    public Quaternion conjugate()
    {
        return new Quaternion(w, -x, -y, -z, this.acquisitionTime);
    }

    /**
     * @deprecated Use {@link #conjugate()} instead.
     */
    @Deprecated
    public Quaternion congugate()
    {
        return conjugate();
    }

    public Quaternion inverse()
    {
        return normalized().conjugate();
    }

    public Quaternion multiply(Quaternion q, long acquisitionTime)
    {
        return new Quaternion(
                this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z,
                this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y,
                this.w * q.y - this.x * q.z + this.y * q.w + this.z * q.x,
                this.w * q.z + this.x * q.y - this.y * q.x + this.z * q.w,
                acquisitionTime);
    }

    /**
     * Apply this rotation to the given vector
     * @param vector The vector to rotate (with XYZ order)
     * @return The rotated vector
     */
    public VectorF applyToVector(VectorF vector)
    {
        // Adapted from https://www.vcalc.com/wiki/vCalc/V3+-+Vector+Rotation under the
        // Creative Common Attribution - ShareAlike License, which all calculators on vCalc are
        // licensed under. See Section 3 of the vCalc License and Terms of Use (as of 2022-10-27):
        // http://web.archive.org/web/20221027024715/https://www.vcalc.com/static/html/terms.html

        float vx = vector.get(0);
        float vy = vector.get(1);
        float vz = vector.get(2);

        float qs = this.w;
        float q1 = this.x;
        float q2 = this.y;
        float q3 = this.z;

        //Compute the inverse rotation quaternion "i"
        float is = qs;
        float i1 = -1*q1;
        float i2 = -1*q2;
        float i3 = -1*q3;

        float S1V2x = qs*vx;
        float S1V2y = qs*vy;
        float S1V2z = qs*vz;

        float V1xV21 = q2*vz - q3*vy;
        float V1xV22 = -1*(q1*vz - q3*vx);
        float V1xV23 = q1*vy - q2*vx;

        float qVs = -1*(q1*vx+q2*vy+q3*vz);
        float qV1 = S1V2x + V1xV21;
        float qV2 = S1V2y + V1xV22;
        float qV3 = S1V2z + V1xV23;

        float TS1V2x = qVs*i1;
        float TS1V2y = qVs*i2;
        float TS1V2z = qVs*i3;

        float TS2V1x = is*qV1;
        float TS2V1y = is*qV2;
        float TS2V1z = is*qV3;

        float TV1XV21 = qV2*i3-qV3*i2;

        float qVq1 = TS1V2x + TS2V1x + TV1XV21;

        float TV1XV22 = -1*(qV1*i3-qV3*i1);
        float qVq2 = TS1V2y + TS2V1y + TV1XV22;

        float TV1XV23 = qV1*i2-qV2*i1;
        float qVq3 = TS1V2z + TS2V1z + TV1XV23;

        return new VectorF(qVq1, qVq2, qVq3);
    }

    public MatrixF toMatrix()
    {
        // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        final float xx = x * x;
        final float xy = x * y;
        final float xz = x * z;
        final float xw = x * w;

        final float yy = y * y;
        final float yz = y * z;
        final float yw = y * w;

        final float zz = z * z;
        final float zw = z * w;

        final float m00 = (float) (1.0 - (2.0 * (yy + zz)));
        final float m01 = (float) (2.0 * (xy - zw));
        final float m02 = (float) (2.0 * (xz + yw));

        final float m10 = (float) (2.0 * (xy + zw));
        final float m11 = (float) (1.0 - (2.0 * (xx + zz)));
        final float m12 = (float) (2.0 * (yz - xw));

        final float m20 = (float) (2.0 * (xz - yw));
        final float m21 = (float) (2.0 * (yz + xw));
        final float m22 = (float) (1.0 - (2.0 * (xx + yy)));

        OpenGLMatrix result = new OpenGLMatrix();
        result.put(0, 0, m00);
        result.put(0, 1, m01);
        result.put(0, 2, m02);
        result.put(1, 0, m10);
        result.put(1, 1, m11);
        result.put(1, 2, m12);
        result.put(2, 0, m20);
        result.put(2, 1, m21);
        result.put(2, 2, m22);

        return result;
    }

    public Orientation toOrientation(AxesReference axesReference, AxesOrder axesOrder, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit)
    {
        Orientation result = Orientation.getOrientation(toMatrix(), axesReference, axesOrder, angleUnit);
        result.acquisitionTime = acquisitionTime;
        return result;
    }

    @Override
    public String toString()
    {
        return String.format(Locale.US, "{w=%.3f, x=%.3f, y=%.3f, z=%.3f}", w, x, y, z);
    }
}
