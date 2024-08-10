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
 * An {@link AngleUnit} represents angles in different units of measure and
 * provides utility methods to convert across units. {@link AngleUnit} does not
 * maintain angle information information internally, but only helps organize
 * and use angle measures that may be maintained separately across various contexts.
 * <p>
 * Angles can be distinguished along (at least) two axes:
 *  <ol>
 *      <li>the fundamental unit (radians vs degrees)</li>
 *      <li>whether the angular quantity is normalized or not to the range of [-180,+180) degrees</li>
 *  </ol>
 *  Normalized angles are of most utility when dealing with physical angles, as normalization
 *  removes ambiguity of representation. In particular, two angles can be compared for equality
 *  by subtracting them, normalizing, and testing whether the absolute value of the result is
 *  smaller than some tolerance threshold. This approach neatly handles all cases of cyclical
 *  wrapping without unexpected discontinuities.
 * <p>
 *  Unnormalized angles can be handy when the angular quantity is not a physical angle but some
 *  related quantity such as an angular <em>velocity</em> or <em>acceleration</em>, where the
 *  quantity in question lacks the 360-degree cyclical equivalence of a physical angle.
 * </p>
 */
@SuppressWarnings("WeakerAccess")
public enum AngleUnit
{
    DEGREES(0), RADIANS(1);
    public final byte bVal;

    protected static final double TwoPi   = 2 * Math.PI;
    public    static final float  Pif     = (float) Math.PI;

    AngleUnit(int i)
    {
        bVal = (byte) i;
    }

    //----------------------------------------------------------------------------------------------
    // Primitive operations
    //----------------------------------------------------------------------------------------------

    public double fromDegrees(double degrees)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return this.normalize(degrees / 180.0 * Math.PI);
            case DEGREES:                   return this.normalize(degrees);
        }
    }

    public float fromDegrees(float degrees)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return this.normalize(degrees / 180.0f * Pif);
            case DEGREES:                   return this.normalize(degrees);
        }
    }

    public double fromRadians(double radians)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return this.normalize(radians);
            case DEGREES:                   return this.normalize(radians / Math.PI * 180.0);
        }
    }

    public float fromRadians(float radians)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return this.normalize(radians);
            case DEGREES:                   return this.normalize(radians / Pif * 180.0f);
        }
    }

    public double fromUnit(AngleUnit them, double theirs)
    {
        switch (them)
        {
            default:
            case RADIANS:                   return this.fromRadians(theirs);
            case DEGREES:                   return this.fromDegrees(theirs);
        }
    }

    public float fromUnit(AngleUnit them, float theirs)
    {
        switch (them)
        {
            default:
            case RADIANS:                   return this.fromRadians(theirs);
            case DEGREES:                   return this.fromDegrees(theirs);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Derived operations
    //----------------------------------------------------------------------------------------------

    public double toDegrees(double inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return DEGREES.fromRadians(inOurUnits);
            case DEGREES:                   return DEGREES.fromDegrees(inOurUnits);
        }
    }

    public float toDegrees(float inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return DEGREES.fromRadians(inOurUnits);
            case DEGREES:                   return DEGREES.fromDegrees(inOurUnits);
        }
    }

    public double toRadians(double inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return RADIANS.fromRadians(inOurUnits);
            case DEGREES:                   return RADIANS.fromDegrees(inOurUnits);
        }
    }

    public float toRadians(float inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return RADIANS.fromRadians(inOurUnits);
            case DEGREES:                   return RADIANS.fromDegrees(inOurUnits);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Normalization
    //----------------------------------------------------------------------------------------------

    public double normalize(double mine)
    {
        switch (this)
        {
            default:
            case RADIANS:               return normalizeRadians(mine);
            case DEGREES:               return normalizeDegrees(mine);
        }
    }

    public float normalize(float mine)
    {
        switch (this)
        {
            default:
            case RADIANS:               return normalizeRadians(mine);
            case DEGREES:               return normalizeDegrees(mine);
        }
    }

    public static double normalizeDegrees(double degrees)
    {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    public static float normalizeDegrees(float degrees)
    {
        return (float)normalizeDegrees((double)degrees);
    }

    public static double normalizeRadians(double radians)
    {
        while (radians >= Math.PI) radians -= TwoPi;
        while (radians < -Math.PI) radians += TwoPi;
        return radians;
    }

    public static float normalizeRadians(float radians)
    {
        return (float)normalizeRadians((double)radians);
    }
}
