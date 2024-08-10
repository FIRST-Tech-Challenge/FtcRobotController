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
import org.firstinspires.ftc.robotcore.internal.system.Assert;

import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Math.asin;
import static java.lang.Math.acos;
import static java.lang.Math.atan2;
import static java.lang.Math.PI;

/**
 * Instances of {@link Orientation} represent a rotated stance in three-dimensional space
 * by way of a set of three successive rotations.
 *
 * <p>There are several ways that a particular orientation in three-space can be represented.
 * One way is by specifying a (unit) directional vector about which the orientation is to occur,
 * together with a rotation angle about that axis. This representation is unique up to the sign of the
 * direction and angle; that is a rotation {@code a} about a vector {@code v} produces the same
 * rotation as a rotation {@code -a} about the vector {@code -v}. While this manner of specifying a
 * rotation is easy to visualize if the vector in question is one of the cardinal axes (ie: X,Y, or Z),
 * many find it more difficult to visualize more complex rotations in this manner.</p>
 *
 * <p>An alternative, more common, way to represent a particular orientation in three-space is by means
 * of indicating three angles of rotation about three successive axes. You might for example be familiar
 * with the notions of heading, elevation, and bank angles for aircraft. Unfortunately, there are 24
 * different yet equivalent ways that a set of three rotational angles about three axes can represent
 * the same effective rotation. As might be expected, this can be the source of much confusion. The
 * 24 different representations break down as follows.</p>
 *
 * <p>First is the matter of the axes reference: is the coordinate system in which the referred-to rotational
 * axes reside a coordinate system that moves with (and so remains fixed relative to) the object being rotated,
 * or do the axes remain fixed relative to the world around the object and are unaffected by the
 * object's rotational motion? The former situation is referred to as an {@link AxesReference#INTRINSIC intrinsic}
 * reference perspective while the latter is an {@link AxesReference#EXTRINSIC extrinsic} perspective.
 * Both points of view are equally valid methodologies, but one or the other may be more understandable
 * or useful in a given application situation.
 * </p>
 *
 * <p>The extrinsic-vs-intrinsic difference accounts for a factor of two in our list of 24 different
 * representations. The remaining factor of 12 breaks down into whether the three rotations all use
 * different axes (and so are a permutation of X, Y, and Z, of which there are six in number), or whether
 * the first and last axes are the same and the middle one different (e.g. Z-Y-Z); this has three
 * choices for the first axis (which is also used for the last) and two remaining choices for the
 * second axis, for a total, again, of six possibilities. The geometry of three-space is such that these
 * twelve choices are the only distinct representational possibilities. As with the extrinsic-vs-
 * intrinsic difference, all twelve of these axis {@link AxesOrder order}s are equally valid ways of
 * indicating orientation, but in any given application, one way may be more useful or easier to
 * understand than another.
 * </p>
 *
 * <p>Even on top of all that, for a given intrinsic-vs-extrinsic distinction, and a given axes
 * ordering, there are two sets of angle rotation that will produce the same orientation. For example,
 * an extrinsic, XZX rotation of (in degrees) 90, -90, 0 is equivalent to an extrinsic, XZX rotation
 * of -90, 90, -180.</p>
 *
 * <p>As was mentioned, much confusion has historically arisen from talking about an orientation as
 * a set of three angles without also clearly indicating which of the 24 representational possibilities
 * one is working within. One aim of {@link Orientation} is to reduce that confusion by being explicitly
 * clear about this issue: an {@link Orientation} always carries along with it the indication of the
 * {@link AxesReference} and {@link AxesOrder} of the orientation. Methods are provided for converting
 * an {@link Orientation} to and from its associated rotation matrix.</p>
 *
 * @see <a href="https://en.wikipedia.org/wiki/Euler_angles">Euler Angles</a>
 * @see <a href="https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation">Axis-Angle Representation</a>
 * @see <a href="https://en.wikipedia.org/wiki/Axes_conventions">Axes Conventions</a>
 * @see <a href="https://en.wikipedia.org/wiki/Rotation_matrix">Rotation Matrix</a>
 */
public class Orientation
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    /**
     * whether we have extrinsic or intrinsic rotations
     *
     * @see #axesOrder
     */
    public AxesReference axesReference;

    /**
     * the order of axes around which our three rotations occur
     *
     * @see #axesReference
     */
    public AxesOrder axesOrder;

    /**
     * the unit in which the angles are expressed
     */
    public org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit;

    /**
     * the chronologically first rotation made in the {@link AxesOrder}
     */
    public float firstAngle;
    /**
     * the chronologically second rotation made in the {@link AxesOrder}
     */
    public float secondAngle;
    /**
     * the chronologically third rotation made in the {@link AxesOrder}
     */
    public float thirdAngle;

    /**
     * the time on the System.nanoTime() clock at which the data was acquired. If no
     * timestamp is associated with this particular set of data, this value is zero.
     */
    public long acquisitionTime;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public Orientation()
    {
        this(AxesReference.EXTRINSIC, AxesOrder.XYZ, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS, 0, 0, 0, 0);
    }

    public Orientation(AxesReference axesReference, AxesOrder axesOrder, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit, float firstAngle, float secondAngle, float thirdAngle, long acquisitionTime)
    {
        this.axesReference = axesReference;
        this.axesOrder = axesOrder;
        this.angleUnit = angleUnit;
        this.firstAngle = firstAngle;
        this.secondAngle = secondAngle;
        this.thirdAngle = thirdAngle;
        this.acquisitionTime = acquisitionTime;
    }

    /**
     * Converts this {@link Orientation} to one with the indicated angular units.
     *
     * @param angleUnit the units to use in the returned [@link Orientation}
     * @return a new [@link Orientation} with the same data but in the indicated units
     */
    public Orientation toAngleUnit(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit)
    {
        if (angleUnit != this.angleUnit)
        {
            return new Orientation(this.axesReference, this.axesOrder, angleUnit,
                    angleUnit.fromUnit(this.angleUnit, firstAngle),
                    angleUnit.fromUnit(this.angleUnit, secondAngle),
                    angleUnit.fromUnit(this.angleUnit, thirdAngle),
                    this.acquisitionTime);
        }
        else
            return this;
    }

    /**
     * Converts the {@link Orientation} to an equivalent one with the indicted point of view.
     *
     * @param axesReference whether we wish to consider rotations from an extrinsic or intrinsic point of view
     * @return an equivalent orientation but with the indicated point of view.
     */
    public Orientation toAxesReference(AxesReference axesReference)
    {
        if (this.axesReference != axesReference)
        {
            /**
             * Theorem: Any extrinsic rotation is equivalent to an intrinsic rotation by
             * the same angles but with inverted order of elemental orientations, and vice versa.
             * @see <a href="https://en.wikipedia.org/wiki/Euler_angles">Euler Angles</a>
             */
            Assert.assertTrue(axesReference == this.axesReference.reverse());
            return new Orientation(this.axesReference.reverse(), this.axesOrder.reverse(), this.angleUnit,
                    this.thirdAngle, this.secondAngle, this.firstAngle, this.acquisitionTime);
        }
        else
            return this;
    }

    /**
     * Converts the {@link Orientation} to an equivalent one with the indicated ordering of axes
     * @param axesOrder the desired ordering of axes
     * @return an equivalent orientation with the indicated axes order
     */
    public Orientation toAxesOrder(AxesOrder axesOrder)
    {
        if (this.axesOrder != axesOrder)
        {
            return Orientation.getOrientation(this.getRotationMatrix(), this.axesReference, axesOrder, this.angleUnit);
        }
        else
            return this;
    }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    @Override public String toString()
    {
        if (this.angleUnit == org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)
            return String.format("{%s %s %.0f %.0f %.0f}", this.axesReference.toString(), this.axesOrder.toString(), this.firstAngle, this.secondAngle, this.thirdAngle);
        else
            return String.format("{%s %s %.3f %.3f %.3f}", this.axesReference.toString(), this.axesOrder.toString(), this.firstAngle, this.secondAngle, this.thirdAngle);
    }

    //----------------------------------------------------------------------------------------------
    // Rotation Matrices
    //----------------------------------------------------------------------------------------------

    /**
     * Returns the rotation matrix associated with the receiver {@link Orientation}.
     *
     * @return the rotation matrix associated with the receiver {@link Orientation}.
     * @see #getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)
     * @see <a href="https://en.wikipedia.org/wiki/Rotation_matrix">Rotation Matrix</a>
     */
    public OpenGLMatrix getRotationMatrix()
    {
        return getRotationMatrix(this.axesReference, this.axesOrder, this.angleUnit, this.firstAngle, this.secondAngle, this.thirdAngle);
    }

    /**
     * Returns the rotation matrix associated with a particular set of three rotational angles.
     *
     * @return the rotation matrix associated with a particular set of three rotational angles.
     * @see #getRotationMatrix()
     * @see <a href="https://en.wikipedia.org/wiki/Rotation_matrix">Rotation Matrix</a>
     */
    public static OpenGLMatrix getRotationMatrix(AxesReference axesReference, AxesOrder axesOrder, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit unit, float firstAngle, float secondAngle, float thirdAngle)
    {
        if (axesReference == AxesReference.INTRINSIC)
        {
            /**
             * Theorem: Any extrinsic rotation is equivalent to an intrinsic rotation by the same
             * angles but with inverted order of elemental orientations, and vice versa.
             * @see <a href="https://en.wikipedia.org/wiki/Euler_angles">Euler Angles</a>
             */
            return getRotationMatrix(axesReference.reverse(), axesOrder.reverse(), unit, thirdAngle, secondAngle, firstAngle);
        }

        /**
         * The extrinsic case takes some work.
         *
         * Implementation note: these computations were created automatically from symbolic Mathematica expressions.
         * Each computes the intrinsic rotation matrix of a given {@link AxesOrder}, where the axes in the {@link AxesOrder}
         * are used left to right chronologically and the angles are applied chronologically as well.
         *
         * For example, the entry for YXZ is rotation matrix of the extrinsic rotation which rotates first
         * Y(firstAngle), then X(secondAngle), and finally Z(thirdAngle). The rotation matrix in this case is
         *      Z(thirdAngle).X(secondAngle).Y(firstAngle)
         * as the matrix order (when *post*-multiplying vectors, as is always done in modern computer graphics systems),
         * which you will notice has the matrices in reverse order from the chronological sequence of extrinsic rotations.
         */

        firstAngle = unit.toRadians(firstAngle);
        secondAngle = unit.toRadians(secondAngle);
        thirdAngle = unit.toRadians(thirdAngle);

        float m00, m01, m02;
        float m10, m11, m12;
        float m20, m21, m22;

        switch (axesOrder)
        {
            default:
            case XZX:
                m00 = ((float) (cos(secondAngle)));
                m01 = ((float) (-(cos(firstAngle) * sin(secondAngle))));
                m02 = ((float) (sin(firstAngle) * sin(secondAngle)));
                m10 = ((float) (cos(thirdAngle) * sin(secondAngle)));
                m11 = ((float) (cos(firstAngle) * cos(secondAngle) * cos(thirdAngle) - sin(firstAngle) * sin(thirdAngle)));
                m12 = ((float) (-(cos(firstAngle) * sin(thirdAngle)) - cos(secondAngle) * cos(thirdAngle) * sin(firstAngle)));
                m20 = ((float) (sin(secondAngle) * sin(thirdAngle)));
                m21 = ((float) (cos(thirdAngle) * sin(firstAngle) + cos(firstAngle) * cos(secondAngle) * sin(thirdAngle)));
                m22 = ((float) (cos(firstAngle) * cos(thirdAngle) - cos(secondAngle) * sin(firstAngle) * sin(thirdAngle)));
                break;
            case XYX:
                m00 = ((float) (cos(secondAngle)));
                m01 = ((float) (sin(firstAngle) * sin(secondAngle)));
                m02 = ((float) (cos(firstAngle) * sin(secondAngle)));
                m10 = ((float) (sin(secondAngle) * sin(thirdAngle)));
                m11 = ((float) (cos(firstAngle) * cos(thirdAngle) - cos(secondAngle) * sin(firstAngle) * sin(thirdAngle)));
                m12 = ((float) (-(cos(firstAngle) * cos(secondAngle) * sin(thirdAngle)) - cos(thirdAngle) * sin(firstAngle)));
                m20 = ((float) (-(cos(thirdAngle) * sin(secondAngle))));
                m21 = ((float) (cos(firstAngle) * sin(thirdAngle) + cos(secondAngle) * cos(thirdAngle) * sin(firstAngle)));
                m22 = ((float) (cos(firstAngle) * cos(secondAngle) * cos(thirdAngle) - sin(firstAngle) * sin(thirdAngle)));
                break;
            case YXY:
                m00 = ((float) (cos(firstAngle) * cos(thirdAngle) - cos(secondAngle) * sin(firstAngle) * sin(thirdAngle)));
                m01 = ((float) (sin(secondAngle) * sin(thirdAngle)));
                m02 = ((float) (cos(thirdAngle) * sin(firstAngle) + cos(firstAngle) * cos(secondAngle) * sin(thirdAngle)));
                m10 = ((float) (sin(firstAngle) * sin(secondAngle)));
                m11 = ((float) (cos(secondAngle)));
                m12 = ((float) (-(cos(firstAngle) * sin(secondAngle))));
                m20 = ((float) (-(cos(firstAngle) * sin(thirdAngle)) - cos(secondAngle) * cos(thirdAngle) * sin(firstAngle)));
                m21 = ((float) (cos(thirdAngle) * sin(secondAngle)));
                m22 = ((float) (cos(firstAngle) * cos(secondAngle) * cos(thirdAngle) - sin(firstAngle) * sin(thirdAngle)));
                break;
            case YZY:
                m00 = ((float) (cos(firstAngle) * cos(secondAngle) * cos(thirdAngle) - sin(firstAngle) * sin(thirdAngle)));
                m01 = ((float) (-(cos(thirdAngle) * sin(secondAngle))));
                m02 = ((float) (cos(firstAngle) * sin(thirdAngle) + cos(secondAngle) * cos(thirdAngle) * sin(firstAngle)));
                m10 = ((float) (cos(firstAngle) * sin(secondAngle)));
                m11 = ((float) (cos(secondAngle)));
                m12 = ((float) (sin(firstAngle) * sin(secondAngle)));
                m20 = ((float) (-(cos(firstAngle) * cos(secondAngle) * sin(thirdAngle)) - cos(thirdAngle) * sin(firstAngle)));
                m21 = ((float) (sin(secondAngle) * sin(thirdAngle)));
                m22 = ((float) (cos(firstAngle) * cos(thirdAngle) - cos(secondAngle) * sin(firstAngle) * sin(thirdAngle)));
                break;
            case ZYZ:
                m00 = ((float) (cos(firstAngle) * cos(secondAngle) * cos(thirdAngle) - sin(firstAngle) * sin(thirdAngle)));
                m01 = ((float) (-(cos(firstAngle) * sin(thirdAngle)) - cos(secondAngle) * cos(thirdAngle) * sin(firstAngle)));
                m02 = ((float) (cos(thirdAngle) * sin(secondAngle)));
                m10 = ((float) (cos(thirdAngle) * sin(firstAngle) + cos(firstAngle) * cos(secondAngle) * sin(thirdAngle)));
                m11 = ((float) (cos(firstAngle) * cos(thirdAngle) - cos(secondAngle) * sin(firstAngle) * sin(thirdAngle)));
                m12 = ((float) (sin(secondAngle) * sin(thirdAngle)));
                m20 = ((float) (-(cos(firstAngle) * sin(secondAngle))));
                m21 = ((float) (sin(firstAngle) * sin(secondAngle)));
                m22 = ((float) (cos(secondAngle)));
                break;
            case ZXZ:
                m00 = ((float) (cos(firstAngle) * cos(thirdAngle) - cos(secondAngle) * sin(firstAngle) * sin(thirdAngle)));
                m01 = ((float) (-(cos(firstAngle) * cos(secondAngle) * sin(thirdAngle)) - cos(thirdAngle) * sin(firstAngle)));
                m02 = ((float) (sin(secondAngle) * sin(thirdAngle)));
                m10 = ((float) (cos(firstAngle) * sin(thirdAngle) + cos(secondAngle) * cos(thirdAngle) * sin(firstAngle)));
                m11 = ((float) (cos(firstAngle) * cos(secondAngle) * cos(thirdAngle) - sin(firstAngle) * sin(thirdAngle)));
                m12 = ((float) (-(cos(thirdAngle) * sin(secondAngle))));
                m20 = ((float) (sin(firstAngle) * sin(secondAngle)));
                m21 = ((float) (cos(firstAngle) * sin(secondAngle)));
                m22 = ((float) (cos(secondAngle)));
                break;
            case XZY:
                m00 = ((float) (cos(secondAngle) * cos(thirdAngle)));
                m01 = ((float) (sin(firstAngle) * sin(thirdAngle) - cos(firstAngle) * cos(thirdAngle) * sin(secondAngle)));
                m02 = ((float) (cos(firstAngle) * sin(thirdAngle) + cos(thirdAngle) * sin(firstAngle) * sin(secondAngle)));
                m10 = ((float) (sin(secondAngle)));
                m11 = ((float) (cos(firstAngle) * cos(secondAngle)));
                m12 = ((float) (-(cos(secondAngle) * sin(firstAngle))));
                m20 = ((float) (-(cos(secondAngle) * sin(thirdAngle))));
                m21 = ((float) (cos(thirdAngle) * sin(firstAngle) + cos(firstAngle) * sin(secondAngle) * sin(thirdAngle)));
                m22 = ((float) (cos(firstAngle) * cos(thirdAngle) - sin(firstAngle) * sin(secondAngle) * sin(thirdAngle)));
                break;
            case XYZ:
                m00 = ((float) (cos(secondAngle) * cos(thirdAngle)));
                m01 = ((float) (cos(thirdAngle) * sin(firstAngle) * sin(secondAngle) - cos(firstAngle) * sin(thirdAngle)));
                m02 = ((float) (sin(firstAngle) * sin(thirdAngle) + cos(firstAngle) * cos(thirdAngle) * sin(secondAngle)));
                m10 = ((float) (cos(secondAngle) * sin(thirdAngle)));
                m11 = ((float) (cos(firstAngle) * cos(thirdAngle) + sin(firstAngle) * sin(secondAngle) * sin(thirdAngle)));
                m12 = ((float) (cos(firstAngle) * sin(secondAngle) * sin(thirdAngle) - cos(thirdAngle) * sin(firstAngle)));
                m20 = ((float) (-sin(secondAngle)));
                m21 = ((float) (cos(secondAngle) * sin(firstAngle)));
                m22 = ((float) (cos(firstAngle) * cos(secondAngle)));
                break;
            case YXZ:
                m00 = ((float) (cos(firstAngle) * cos(thirdAngle) - sin(firstAngle) * sin(secondAngle) * sin(thirdAngle)));
                m01 = ((float) (-(cos(secondAngle) * sin(thirdAngle))));
                m02 = ((float) (cos(thirdAngle) * sin(firstAngle) + cos(firstAngle) * sin(secondAngle) * sin(thirdAngle)));
                m10 = ((float) (cos(firstAngle) * sin(thirdAngle) + cos(thirdAngle) * sin(firstAngle) * sin(secondAngle)));
                m11 = ((float) (cos(secondAngle) * cos(thirdAngle)));
                m12 = ((float) (sin(firstAngle) * sin(thirdAngle) - cos(firstAngle) * cos(thirdAngle) * sin(secondAngle)));
                m20 = ((float) (-(cos(secondAngle) * sin(firstAngle))));
                m21 = ((float) (sin(secondAngle)));
                m22 = ((float) (cos(firstAngle) * cos(secondAngle)));
                break;
            case YZX:
                m00 = ((float) (cos(firstAngle) * cos(secondAngle)));
                m01 = ((float) (-sin(secondAngle)));
                m02 = ((float) (cos(secondAngle) * sin(firstAngle)));
                m10 = ((float) (sin(firstAngle) * sin(thirdAngle) + cos(firstAngle) * cos(thirdAngle) * sin(secondAngle)));
                m11 = ((float) (cos(secondAngle) * cos(thirdAngle)));
                m12 = ((float) (cos(thirdAngle) * sin(firstAngle) * sin(secondAngle) - cos(firstAngle) * sin(thirdAngle)));
                m20 = ((float) (cos(firstAngle) * sin(secondAngle) * sin(thirdAngle) - cos(thirdAngle) * sin(firstAngle)));
                m21 = ((float) (cos(secondAngle) * sin(thirdAngle)));
                m22 = ((float) (cos(firstAngle) * cos(thirdAngle) + sin(firstAngle) * sin(secondAngle) * sin(thirdAngle)));
                break;
            case ZYX:
                m00 = ((float) (cos(firstAngle) * cos(secondAngle)));
                m01 = ((float) (-(cos(secondAngle) * sin(firstAngle))));
                m02 = ((float) (sin(secondAngle)));
                m10 = ((float) (cos(thirdAngle) * sin(firstAngle) + cos(firstAngle) * sin(secondAngle) * sin(thirdAngle)));
                m11 = ((float) (cos(firstAngle) * cos(thirdAngle) - sin(firstAngle) * sin(secondAngle) * sin(thirdAngle)));
                m12 = ((float) (-(cos(secondAngle) * sin(thirdAngle))));
                m20 = ((float) (sin(firstAngle) * sin(thirdAngle) - cos(firstAngle) * cos(thirdAngle) * sin(secondAngle)));
                m21 = ((float) (cos(firstAngle) * sin(thirdAngle) + cos(thirdAngle) * sin(firstAngle) * sin(secondAngle)));
                m22 = ((float) (cos(secondAngle) * cos(thirdAngle)));
                break;
            case ZXY:
                m00 = ((float) (cos(firstAngle) * cos(thirdAngle) + sin(firstAngle) * sin(secondAngle) * sin(thirdAngle)));
                m01 = ((float) (cos(firstAngle) * sin(secondAngle) * sin(thirdAngle) - cos(thirdAngle) * sin(firstAngle)));
                m02 = ((float) (cos(secondAngle) * sin(thirdAngle)));
                m10 = ((float) (cos(secondAngle) * sin(firstAngle)));
                m11 = ((float) (cos(firstAngle) * cos(secondAngle)));
                m12 = ((float) (-sin(secondAngle)));
                m20 = ((float) (cos(thirdAngle) * sin(firstAngle) * sin(secondAngle) - cos(firstAngle) * sin(thirdAngle)));
                m21 = ((float) (sin(firstAngle) * sin(thirdAngle) + cos(firstAngle) * cos(thirdAngle) * sin(secondAngle)));
                m22 = ((float) (cos(secondAngle) * cos(thirdAngle)));
                break;
        }

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

    /**
     * Given a rotation matrix, and an {@link AxesReference} and {@link AxesOrder}, returns an orientation
     * that would produce that rotation matrix.
     *
     * @param rot           the matrix whose orientation is to be determined
     * @param axesReference whether wish an extrinsic or intrinsic reference for the axes
     * @param axesOrder     the order in which the axes are to be rotated
     * @param unit          the angle units in which the orientation is to be returned
     * @return an orientation that will produce the given rotation matrix
     * @see Orientation
     * @see #getOrientation(MatrixF, AxesReference, AxesOrder, AngleUnit, AngleSet)
     * @see <a href="https://en.wikipedia.org/wiki/Rotation_matrix">Rotation Matrix</a>
     */
    public static Orientation getOrientation(MatrixF rot, AxesReference axesReference, AxesOrder axesOrder, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit unit)
    {
        /**
         * Run both choices and return the one that uses smaller sets of angles. This is just a heuristic
         * to choose which angle set is the most aesthetically pleasing. Both angle sets are equally <em>valid</em>.
         */
        Orientation one      = getOrientation(rot, axesReference, axesOrder, unit, AngleSet.THEONE);
        Orientation theOther = getOrientation(rot, axesReference, axesOrder, unit, AngleSet.THEOTHER);

        VectorF vOne = new VectorF(one.firstAngle, one.secondAngle, one.thirdAngle);
        VectorF vOther = new VectorF(theOther.firstAngle, theOther.secondAngle, theOther.thirdAngle);

        return vOne.magnitude() <= vOther.magnitude() ? one : theOther;
    }

    /**
     * {@link AngleSet} is used to distinguish between the two sets of angles that will produce
     * a given rotation in a given axes reference and a given axes order
     */
    public enum AngleSet { THEONE, THEOTHER };

    /**
     * Given a rotation matrix, and an {@link AxesReference} and {@link AxesOrder}, returns an orientation
     * that would produce that rotation matrix.
     *
     * @param rot           the matrix whose orientation is to be determined
     * @param axesReference whether wish an extrinsic or intrinsic reference for the axes
     * @param axesOrder     the order in which the axes are to be rotated
     * @param unit          the angle units in which the orientation is to be returned
     * @param angleSet      which of the two sets angles which can produce the rotation matrix is desired
     * @return an orientation that will produce the given rotation matrix
     * @see #getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)
     * @see <a href="https://en.wikipedia.org/wiki/Rotation_matrix">Rotation Matrix</a>
     */
    public static Orientation getOrientation(MatrixF rot, AxesReference axesReference, AxesOrder axesOrder, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit unit, AngleSet angleSet)
    {
        float firstAngle, secondAngle, thirdAngle;

        if (axesReference == AxesReference.INTRINSIC)
        {
            return getOrientation(rot, axesReference.reverse(), axesOrder.reverse(), unit, angleSet).toAxesReference(axesReference);
        }

        /**
         * The extrinsic case takes some work.
         *
         * Implementation note: these computations contained in this 'switch' were derived automatically
         * from symbolic Mathematica representations of the corresponding twelve forms of rotation
         * matrix. The output of that automatic processing was literally copied and pasted from Mathematica
         * into this Java source without intervention of human editing (save for reformatting of the code)
         * thus significantly reducing the chance that errors might be inadvertently introduced.
         *
         * The cases labelled here as "arbitrary" are situations in which
         * <a href="https://en.wikipedia.org/wiki/Gimbal_lock">gimbal lock</a> occurs. In those situations,
         * the three angles are not uniquely specified by the rotation. Instead, only the sum or difference
         * (as the case may be) of two of the axes is determined. In those situations, we make an
         * arbitrary choice for the value of one of those two axes, then appropriately compute the other.
         */
        float test;
        switch (axesOrder)
        {
            default:
            case XZX:
                test = rot.get(0, 0);  /*  cos(secondAngle)  */
                if (test == 1)
                {
                    secondAngle = (float) 0;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(2, 1) == sin(firstAngle + thirdAngle)  */
                    /*  rot.get(1, 1) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(rot.get(2, 1), rot.get(1, 1)) - firstAngle);
                }
                else if (test == -1)
                {
                    secondAngle = (float) PI;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(1, 2) == sin(firstAngle - thirdAngle)  */
                    /*  rot.get(2, 2) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(rot.get(1, 2), rot.get(2, 2)));
                }
                else
                {
                    /*  rot.get(0, 0) == cos(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? acos(rot.get(0, 0)) : -acos(rot.get(0, 0)));
                    /*  rot.get(0, 2) == sin(firstAngle) * sin(secondAngle)  */
                    /*  rot.get(0, 1) == -(cos(firstAngle) * sin(secondAngle))  */
                    firstAngle = (float) atan2(rot.get(0, 2) / sin(secondAngle), -rot.get(0, 1) / sin(secondAngle));
                    /*  rot.get(2, 0) == sin(secondAngle) * sin(thirdAngle)  */
                    /*  rot.get(1, 0) == cos(thirdAngle) * sin(secondAngle)  */
                    thirdAngle = (float) atan2(rot.get(2, 0) / sin(secondAngle), rot.get(1, 0) / sin(secondAngle));
                }
                break;
            case XYX:
                test = rot.get(0, 0);  /*  cos(secondAngle)  */
                if (test == 1)
                {
                    secondAngle = (float) 0;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(2, 1) == sin(firstAngle + thirdAngle)  */
                    /*  rot.get(1, 1) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(rot.get(2, 1), rot.get(1, 1)) - firstAngle);
                }
                else if (test == -1)
                {
                    secondAngle = (float) PI;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(1, 2) == -sin(firstAngle - thirdAngle)  */
                    /*  rot.get(1, 1) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(-rot.get(1, 2), rot.get(1, 1)));
                }
                else
                {
                    /*  rot.get(0, 0) == cos(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? acos(rot.get(0, 0)) : -acos(rot.get(0, 0)));
                    /*  rot.get(0, 1) == sin(firstAngle) * sin(secondAngle)  */
                    /*  rot.get(0, 2) == cos(firstAngle) * sin(secondAngle)  */
                    firstAngle = (float) atan2(rot.get(0, 1) / sin(secondAngle), rot.get(0, 2) / sin(secondAngle));
                    /*  rot.get(1, 0) == sin(secondAngle) * sin(thirdAngle)  */
                    /*  rot.get(2, 0) == -(cos(thirdAngle) * sin(secondAngle))  */
                    thirdAngle = (float) atan2(rot.get(1, 0) / sin(secondAngle), -rot.get(2, 0) / sin(secondAngle));
                }
                break;
            case YXY:
                test = rot.get(1, 1);  /*  cos(secondAngle)  */
                if (test == 1)
                {
                    secondAngle = (float) 0;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 2) == sin(firstAngle + thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(rot.get(0, 2), rot.get(0, 0)) - firstAngle);
                }
                else if (test == -1)
                {
                    secondAngle = (float) PI;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 2) == sin(firstAngle - thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(rot.get(0, 2), rot.get(0, 0)));
                }
                else
                {
                    /*  rot.get(1, 1) == cos(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? acos(rot.get(1, 1)) : -acos(rot.get(1, 1)));
                    /*  rot.get(1, 0) == sin(firstAngle) * sin(secondAngle)  */
                    /*  rot.get(1, 2) == -(cos(firstAngle) * sin(secondAngle))  */
                    firstAngle = (float) atan2(rot.get(1, 0) / sin(secondAngle), -rot.get(1, 2) / sin(secondAngle));
                    /*  rot.get(0, 1) == sin(secondAngle) * sin(thirdAngle)  */
                    /*  rot.get(2, 1) == cos(thirdAngle) * sin(secondAngle)  */
                    thirdAngle = (float) atan2(rot.get(0, 1) / sin(secondAngle), rot.get(2, 1) / sin(secondAngle));
                }
                break;
            case YZY:
                test = rot.get(1, 1);  /*  cos(secondAngle)  */
                if (test == 1)
                {
                    secondAngle = (float) 0;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 2) == sin(firstAngle + thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(rot.get(0, 2), rot.get(0, 0)) - firstAngle);
                }
                else if (test == -1)
                {
                    secondAngle = (float) PI;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 2) == -sin(firstAngle - thirdAngle)  */
                    /*  rot.get(2, 2) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(-rot.get(0, 2), rot.get(2, 2)));
                }
                else
                {
                    /*  rot.get(1, 1) == cos(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? acos(rot.get(1, 1)) : -acos(rot.get(1, 1)));
                    /*  rot.get(1, 2) == sin(firstAngle) * sin(secondAngle)  */
                    /*  rot.get(1, 0) == cos(firstAngle) * sin(secondAngle)  */
                    firstAngle = (float) atan2(rot.get(1, 2) / sin(secondAngle), rot.get(1, 0) / sin(secondAngle));
                    /*  rot.get(2, 1) == sin(secondAngle) * sin(thirdAngle)  */
                    /*  rot.get(0, 1) == -(cos(thirdAngle) * sin(secondAngle))  */
                    thirdAngle = (float) atan2(rot.get(2, 1) / sin(secondAngle), -rot.get(0, 1) / sin(secondAngle));
                }
                break;
            case ZYZ:
                test = rot.get(2, 2);  /*  cos(secondAngle)  */
                if (test == 1)
                {
                    secondAngle = (float) 0;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(1, 0) == sin(firstAngle + thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(rot.get(1, 0), rot.get(0, 0)) - firstAngle);
                }
                else if (test == -1)
                {
                    secondAngle = (float) PI;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 1) == sin(firstAngle - thirdAngle)  */
                    /*  rot.get(1, 1) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(rot.get(0, 1), rot.get(1, 1)));
                }
                else
                {
                    /*  rot.get(2, 2) == cos(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? acos(rot.get(2, 2)) : -acos(rot.get(2, 2)));
                    /*  rot.get(2, 1) == sin(firstAngle) * sin(secondAngle)  */
                    /*  rot.get(2, 0) == -(cos(firstAngle) * sin(secondAngle))  */
                    firstAngle = (float) atan2(rot.get(2, 1) / sin(secondAngle), -rot.get(2, 0) / sin(secondAngle));
                    /*  rot.get(1, 2) == sin(secondAngle) * sin(thirdAngle)  */
                    /*  rot.get(0, 2) == cos(thirdAngle) * sin(secondAngle)  */
                    thirdAngle = (float) atan2(rot.get(1, 2) / sin(secondAngle), rot.get(0, 2) / sin(secondAngle));
                }
                break;
            case ZXZ:
                test = rot.get(2, 2);  /*  cos(secondAngle)  */
                if (test == 1)
                {
                    secondAngle = (float) 0;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(1, 0) == sin(firstAngle + thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(rot.get(1, 0), rot.get(0, 0)) - firstAngle);
                }
                else if (test == -1)
                {
                    secondAngle = (float) PI;
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 1) == -sin(firstAngle - thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(-rot.get(0, 1), rot.get(0, 0)));
                }
                else
                {
                    /*  rot.get(2, 2) == cos(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? acos(rot.get(2, 2)) : -acos(rot.get(2, 2)));
                    /*  rot.get(2, 0) == sin(firstAngle) * sin(secondAngle)  */
                    /*  rot.get(2, 1) == cos(firstAngle) * sin(secondAngle)  */
                    firstAngle = (float) atan2(rot.get(2, 0) / sin(secondAngle), rot.get(2, 1) / sin(secondAngle));
                    /*  rot.get(0, 2) == sin(secondAngle) * sin(thirdAngle)  */
                    /*  rot.get(1, 2) == -(cos(thirdAngle) * sin(secondAngle))  */
                    thirdAngle = (float) atan2(rot.get(0, 2) / sin(secondAngle), -rot.get(1, 2) / sin(secondAngle));
                }
                break;
            case XZY:
                test = rot.get(1, 0);  /*  sin(secondAngle)  */
                if (test == 1)
                {
                    secondAngle = (float) (PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 2) == sin(firstAngle + thirdAngle)  */
                    /*  rot.get(2, 2) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(rot.get(0, 2), rot.get(2, 2)) - firstAngle);
                }
                else if (test == -1)
                {
                    secondAngle = (float) (-PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(2, 1) == sin(firstAngle - thirdAngle)  */
                    /*  rot.get(0, 1) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(rot.get(2, 1), rot.get(0, 1)));
                }
                else
                {
                    /*  rot.get(1, 0) == sin(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? asin(rot.get(1, 0)) : PI - asin(rot.get(1, 0)));
                    /*  rot.get(1, 2) == -(cos(secondAngle) * sin(firstAngle))  */
                    /*  rot.get(1, 1) == cos(firstAngle) * cos(secondAngle)  */
                    firstAngle = (float) atan2(-rot.get(1, 2) / cos(secondAngle), rot.get(1, 1) / cos(secondAngle));
                    /*  rot.get(2, 0) == -(cos(secondAngle) * sin(thirdAngle))  */
                    /*  rot.get(0, 0) == cos(secondAngle) * cos(thirdAngle)  */
                    thirdAngle = (float) atan2(-rot.get(2, 0) / cos(secondAngle), rot.get(0, 0) / cos(secondAngle));
                }
                break;
            case XYZ:
                test = rot.get(2, 0);  /*  -sin(secondAngle)  */
                if (test == -1)
                {
                    secondAngle = (float) (PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 1) == sin(firstAngle - thirdAngle)  */
                    /*  rot.get(0, 2) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(rot.get(0, 1), rot.get(0, 2)));
                }
                else if (test == 1)
                {
                    secondAngle = (float) (-PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 1) == -sin(firstAngle + thirdAngle)  */
                    /*  rot.get(1, 1) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(-rot.get(0, 1), rot.get(1, 1)) - firstAngle);
                }
                else
                {
                    /*  rot.get(2, 0) == -sin(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? -asin(rot.get(2, 0)) : PI + asin(rot.get(2, 0)));
                    /*  rot.get(2, 1) == cos(secondAngle) * sin(firstAngle)  */
                    /*  rot.get(2, 2) == cos(firstAngle) * cos(secondAngle)  */
                    firstAngle = (float) atan2(rot.get(2, 1) / cos(secondAngle), rot.get(2, 2) / cos(secondAngle));
                    /*  rot.get(1, 0) == cos(secondAngle) * sin(thirdAngle)  */
                    /*  rot.get(0, 0) == cos(secondAngle) * cos(thirdAngle)  */
                    thirdAngle = (float) atan2(rot.get(1, 0) / cos(secondAngle), rot.get(0, 0) / cos(secondAngle));
                }
                break;
            case YXZ:
                test = rot.get(2, 1);  /*  sin(secondAngle)  */
                if (test == 1)
                {
                    secondAngle = (float) (PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 2) == sin(firstAngle + thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(rot.get(0, 2), rot.get(0, 0)) - firstAngle);
                }
                else if (test == -1)
                {
                    secondAngle = (float) (-PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 2) == sin(firstAngle - thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(rot.get(0, 2), rot.get(0, 0)));
                }
                else
                {
                    /*  rot.get(2, 1) == sin(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? asin(rot.get(2, 1)) : PI - asin(rot.get(2, 1)));
                    /*  rot.get(2, 0) == -(cos(secondAngle) * sin(firstAngle))  */
                    /*  rot.get(2, 2) == cos(firstAngle) * cos(secondAngle)  */
                    firstAngle = (float) atan2(-rot.get(2, 0) / cos(secondAngle), rot.get(2, 2) / cos(secondAngle));
                    /*  rot.get(0, 1) == -(cos(secondAngle) * sin(thirdAngle))  */
                    /*  rot.get(1, 1) == cos(secondAngle) * cos(thirdAngle)  */
                    thirdAngle = (float) atan2(-rot.get(0, 1) / cos(secondAngle), rot.get(1, 1) / cos(secondAngle));
                }
                break;
            case YZX:
                test = rot.get(0, 1);  /*  -sin(secondAngle)  */
                if (test == -1)
                {
                    secondAngle = (float) (PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(1, 2) == sin(firstAngle - thirdAngle)  */
                    /*  rot.get(1, 0) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(rot.get(1, 2), rot.get(1, 0)));
                }
                else if (test == 1)
                {
                    secondAngle = (float) (-PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(1, 2) == -sin(firstAngle + thirdAngle)  */
                    /*  rot.get(2, 2) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(-rot.get(1, 2), rot.get(2, 2)) - firstAngle);
                }
                else
                {
                    /*  rot.get(0, 1) == -sin(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? -asin(rot.get(0, 1)) : PI + asin(rot.get(0, 1)));
                    /*  rot.get(0, 2) == cos(secondAngle) * sin(firstAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle) * cos(secondAngle)  */
                    firstAngle = (float) atan2(rot.get(0, 2) / cos(secondAngle), rot.get(0, 0) / cos(secondAngle));
                    /*  rot.get(2, 1) == cos(secondAngle) * sin(thirdAngle)  */
                    /*  rot.get(1, 1) == cos(secondAngle) * cos(thirdAngle)  */
                    thirdAngle = (float) atan2(rot.get(2, 1) / cos(secondAngle), rot.get(1, 1) / cos(secondAngle));
                }
                break;
            case ZYX:
                test = rot.get(0, 2);  /*  sin(secondAngle)  */
                if (test == 1)
                {
                    secondAngle = (float) (PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(1, 0) == sin(firstAngle + thirdAngle)  */
                    /*  rot.get(1, 1) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(rot.get(1, 0), rot.get(1, 1)) - firstAngle);
                }
                else if (test == -1)
                {
                    secondAngle = (float) (-PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(1, 0) == sin(firstAngle - thirdAngle)  */
                    /*  rot.get(1, 1) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(rot.get(1, 0), rot.get(1, 1)));
                }
                else
                {
                    /*  rot.get(0, 2) == sin(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? asin(rot.get(0, 2)) : PI - asin(rot.get(0, 2)));
                    /*  rot.get(0, 1) == -(cos(secondAngle) * sin(firstAngle))  */
                    /*  rot.get(0, 0) == cos(firstAngle) * cos(secondAngle)  */
                    firstAngle = (float) atan2(-rot.get(0, 1) / cos(secondAngle), rot.get(0, 0) / cos(secondAngle));
                    /*  rot.get(1, 2) == -(cos(secondAngle) * sin(thirdAngle))  */
                    /*  rot.get(2, 2) == cos(secondAngle) * cos(thirdAngle)  */
                    thirdAngle = (float) atan2(-rot.get(1, 2) / cos(secondAngle), rot.get(2, 2) / cos(secondAngle));
                }
                break;
            case ZXY:
                test = rot.get(1, 2);  /*  -sin(secondAngle)  */
                if (test == -1)
                {
                    secondAngle = (float) (PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(2, 0) == sin(firstAngle - thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle - thirdAngle)  */
                    thirdAngle = (float) (firstAngle - atan2(rot.get(2, 0), rot.get(0, 0)));
                }
                else if (test == 1)
                {
                    secondAngle = (float) (-PI / 2);
                    firstAngle = (float) 0;  /*  arbitrary  */
                    /*  rot.get(0, 1) == -sin(firstAngle + thirdAngle)  */
                    /*  rot.get(0, 0) == cos(firstAngle + thirdAngle)  */
                    thirdAngle = (float) (atan2(-rot.get(0, 1), rot.get(0, 0)) - firstAngle);
                }
                else
                {
                    /*  rot.get(1, 2) == -sin(secondAngle)  */
                    secondAngle = (float) (angleSet == AngleSet.THEONE ? -asin(rot.get(1, 2)) : PI + asin(rot.get(1, 2)));
                    /*  rot.get(1, 0) == cos(secondAngle) * sin(firstAngle)  */
                    /*  rot.get(1, 1) == cos(firstAngle) * cos(secondAngle)  */
                    firstAngle = (float) atan2(rot.get(1, 0) / cos(secondAngle), rot.get(1, 1) / cos(secondAngle));
                    /*  rot.get(0, 2) == cos(secondAngle) * sin(thirdAngle)  */
                    /*  rot.get(2, 2) == cos(secondAngle) * cos(thirdAngle)  */
                    thirdAngle = (float) atan2(rot.get(0, 2) / cos(secondAngle), rot.get(2, 2) / cos(secondAngle));
                }
                break;
        }

        return new Orientation(axesReference, axesOrder, unit,
                unit.fromRadians(firstAngle), unit.fromRadians(secondAngle), unit.fromRadians(thirdAngle),
                0);
    }

}

