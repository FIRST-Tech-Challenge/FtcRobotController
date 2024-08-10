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
 * {@link DistanceUnit} represents a unit of measure of distance.
 */
public enum DistanceUnit
{
    METER(0), CM(1), MM(2), INCH(3);
    public final byte bVal;

    public static final double infinity = Double.MAX_VALUE;
    public static final double mmPerInch = 25.4;
    public static final double mPerInch = mmPerInch * 0.001;

    DistanceUnit(int i)
    {
        this.bVal = (byte)i;
    }

    //----------------------------------------------------------------------------------------------
    // Primitive operations
    //----------------------------------------------------------------------------------------------

    public double fromMeters(double meters)
    {
        if (meters==infinity) return infinity;
        switch (this)
        {
            default:
            case METER:   return meters;
            case CM:      return meters * 100;
            case MM:      return meters * 1000;
            case INCH:    return meters / mPerInch;
        }
    }

    public double fromInches(double inches)
    {
        if (inches==infinity) return infinity;
        switch (this)
        {
            default:
            case METER:   return inches * mPerInch;
            case CM:      return inches * mPerInch * 100;
            case MM:      return inches * mPerInch * 1000;
            case INCH:    return inches;
        }
    }

    public double fromCm(double cm)
    {
        if (cm==infinity) return infinity;
        switch (this)
        {
            default:
            case METER:   return cm / 100;
            case CM:      return cm;
            case MM:      return cm * 10;
            case INCH:    return fromMeters(METER.fromCm(cm));
        }
    }

    public double fromMm(double mm)
    {
        if (mm==infinity) return infinity;
        switch (this)
        {
            default:
            case METER:   return mm / 1000;
            case CM:      return mm / 10;
            case MM:      return mm;
            case INCH:    return fromMeters(METER.fromMm(mm));
        }
    }

    public double fromUnit(DistanceUnit him, double his)
    {
        switch (him)
        {
            default:
            case METER:   return this.fromMeters(his);
            case CM:      return this.fromCm(his);
            case MM:      return this.fromMm(his);
            case INCH:    return this.fromInches(his);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Derived operations
    //----------------------------------------------------------------------------------------------

    public double toMeters(double inOurUnits)
    {
        switch (this)
        {
            default:
            case METER:   return METER.fromMeters(inOurUnits);
            case CM:      return METER.fromCm(inOurUnits);
            case MM:      return METER.fromMm(inOurUnits);
            case INCH:    return METER.fromInches(inOurUnits);
        }
    }

    public double toInches(double inOurUnits)
    {
        switch (this)
        {
            default:
            case METER:   return INCH.fromMeters(inOurUnits);
            case CM:      return INCH.fromCm(inOurUnits);
            case MM:      return INCH.fromMm(inOurUnits);
            case INCH:    return INCH.fromInches(inOurUnits);
        }
    }

    public double toCm(double inOurUnits)
    {
        switch (this)
        {
            default:
            case METER:   return CM.fromMeters(inOurUnits);
            case CM:      return CM.fromCm(inOurUnits);
            case MM:      return CM.fromMm(inOurUnits);
            case INCH:    return CM.fromInches(inOurUnits);
        }
    }

    public double toMm(double inOurUnits)
    {
        switch (this)
        {
            default:
            case METER:   return MM.fromMeters(inOurUnits);
            case CM:      return MM.fromCm(inOurUnits);
            case MM:      return MM.fromMm(inOurUnits);
            case INCH:    return MM.fromInches(inOurUnits);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    public String toString(double inOurUnits)
    {
        switch (this)
        {
            default:
            case METER:   return String.format(Locale.getDefault(), "%.3fm", inOurUnits);
            case CM:      return String.format(Locale.getDefault(), "%.1fcm", inOurUnits);
            case MM:      return String.format(Locale.getDefault(), "%.0fmm", inOurUnits);
            case INCH:    return String.format(Locale.getDefault(), "%.2fin", inOurUnits);
        }
    }

    @Override public String toString()
    {
        switch (this)
        {
            default:
            case METER:   return "m";
            case CM:      return "cm";
            case MM:      return "mm";
            case INCH:    return "in";
        }
    }
}

