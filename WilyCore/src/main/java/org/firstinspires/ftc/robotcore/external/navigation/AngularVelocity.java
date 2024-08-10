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

import java.util.Locale;

/**
 * Instances of {@link AngularVelocity} represent an instantaneous body-referenced 3D rotation rate.
 *
 * <p>The instantaneous rate of change of an {@link Orientation}, which is what we are representing
 * here, has unexpected subtleties. As described in Section 9.3 of the MIT Kinematics Lecture below,
 * the <em>instantaneous</em> body-referenced rotation rate angles are decoupled (their order does not
 * matter) but their conversion into a corresponding instantaneous rate of change of a set of
 * related Euler angles (ie: {@link Orientation} involves a non-obvious transformation on two
 * of the rotation rates.</p>
 *
 * @see <a href="http://ocw.mit.edu/courses/mechanical-engineering/2-017j-design-of-electromechanical-robotic-systems-fall-2009/course-text/MIT2_017JF09_ch09.pdf">MIT Kinematics Lecture</a>
 * @see <a href="http://www.chrobotics.com/library/understanding-euler-angles">Understanding Euler Angles</a>
 * @see <a href="https://en.wikipedia.org/wiki/Angular_velocity">Angular Velocity</a>
 */
public class AngularVelocity
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    /**
     * the angular unit in which angular rates are expressed. The time unit thereof
     * is always "per second".
     */
    public AngleUnit unit;

    /**
     * the instantaneous body-referenced rotation rate about the x-axis in units
     * of "{@link #unit}s per second".
     */
    public float xRotationRate;
    /**
     * the instantaneous body-referenced rotation rate about the y-axis in units
     * of "{@link #unit}s per second".
     */
    public float yRotationRate;
    /**
     * the instantaneous body-referenced rotation rate about the z-axis in units
     * of "{@link #unit}s per second".
     */
    public float zRotationRate;

    /**
     * the time on the System.nanoTime() clock at which the data was acquired. If no
     * timestamp is associated with this particular set of data, this value is zero.
     */
    public long acquisitionTime;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public AngularVelocity()
    {
        this(AngleUnit.DEGREES, 0, 0, 0, 0);
    }

    public AngularVelocity(AngleUnit unit, float xRotationRate, float yRotationRate, float zRotationRate, long acquisitionTime)
    {
        this.unit = unit;
        this.xRotationRate = xRotationRate;
        this.yRotationRate = yRotationRate;
        this.zRotationRate = zRotationRate;
        this.acquisitionTime = acquisitionTime;
    }

    /**
     * Converts this {@link AngularVelocity} to one with the indicated angular units.
     *
     * @param unit the units to use in the returned [@link AngularVelocity}
     * @return a new [@link AngularVelocity} with the same data but in the indicated units
     */
    public AngularVelocity toAngleUnit(AngleUnit unit)
    {
        if (unit != this.unit)
        {
            return new AngularVelocity(unit,
                    unit.fromUnit(this.unit, xRotationRate),
                    unit.fromUnit(this.unit, yRotationRate),
                    unit.fromUnit(this.unit, zRotationRate),
                    this.acquisitionTime);
        }
        else
            return this;
    }

    @Override
    public String toString()
    {
        return String.format(Locale.US, "{x=%.3f, y=%.3f, z=%.3f (%s)}", xRotationRate, yRotationRate, zRotationRate, unit);
    }
}
