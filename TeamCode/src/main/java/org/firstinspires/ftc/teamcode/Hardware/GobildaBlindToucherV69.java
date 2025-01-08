/*
Copyright (c) 2019 REV Robotics LLC

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of REV Robotics LLC nor the names of his contributors may be used to
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
package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.NarrowOrgColorSensorImpl;

import java.nio.ByteOrder;
import java.util.Locale;

/**
 * {@link } implements support for the REV Robotics Color Sensor V3.
 *
 * @see <a href="http://revrobotics.com">REV Robotics Website</a>
 *
 */
@SuppressWarnings("WeakerAccess")
@I2cDeviceType
@DeviceProperties(name = "GobildaBlindToucherV69",
        description = "bruh",
        xmlTag = "GobildaBlindToucherV69")
public class GobildaBlindToucherV69 extends NarrowOrgColorSensorImpl implements DistanceSensor, OpticalDistanceSensor, ColorRangeSensor
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected static final double apiLevelMin = 0.0;
    protected static final double apiLevelMax = 1.0;

    /**
     * Experimentally determined constants for converting optical measurements to distance.
     */
    double aParam = 325.961;
    double binvParam = -0.75934;
    double cParam = 26.980;
    double maxDist = 6.0; // inches

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public GobildaBlindToucherV69(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned)
    {
        super(GobildaBlindToucherV69.Parameters.createForAPDS9151(), deviceClient, deviceClientIsOwned);
    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    @Override
    public String getDeviceName()
    {
        return "GobildaBlindToucherV69";
    }

    @Override
    public HardwareDevice.Manufacturer getManufacturer()
    {
        return Manufacturer.Broadcom;
    }

    //----------------------------------------------------------------------------------------------
    // OpticalDistanceSensor / LightSensor
    //----------------------------------------------------------------------------------------------

    @Override public double getLightDetected()
    {
        return Range.clip(
                Range.scale(getRawLightDetected(), 0, getRawLightDetectedMax(), apiLevelMin, apiLevelMax),
                apiLevelMin, apiLevelMax);
    }

    @Override public double getRawLightDetected()
    {
        return rawOptical();
    }

    @Override public double getRawLightDetectedMax()
    {
        return parameters.proximitySaturation;
    }

    @Override public String status()
    {
        return String.format(Locale.getDefault(), "%s on %s", getDeviceName(), getConnectionInfo());
    }

    //----------------------------------------------------------------------------------------------
    // DistanceSensor
    //----------------------------------------------------------------------------------------------

    /**
     * Returns a calibrated, linear sense of distance as read by the infrared proximity
     * part of the sensor. Distance is measured to the plastic housing at the front of the
     * sensor.
     *
     * Natively, the raw optical signal follows an inverse square law. Here, parameters have
     * been fitted to turn that into a <em>linear</em> measure of distance. The function fitted
     * was of the form:
     *
     *      RawOptical = a * distance^b + c
     *
     * The calibration was performed with proximity sensor pulses set to 32, LED driver current set
     * to 125ma, and measurement rate set to 100ms. If the end user chooses to use different
     * settings, the device will need to be recalibrated.
     *
     * Additionally, calibration was performed using card stock measured head on. Actual raw values
     * measured will depend on reflectivity and angle of surface. End user should experimentally
     * verify calibration is appropriate for their own application.
     *
     * @param unit  the unit of distance in which the result should be returned
     * @return      the currently measured distance in the indicated units. will always be between
     *              0.25 and 6.0 inches.
     */
    @Override public double getDistance(DistanceUnit unit)
    {
        int rawOptical = rawOptical();
        double inOptical = inFromOptical(rawOptical);
        return unit.fromUnit(DistanceUnit.INCH, inOptical);
    }

    /**
     * Converts a raw optical inverse-square reading into a fitted, calibrated linear reading in
     * INCHES.
     */
    protected double inFromOptical(int rawOptical)
    {
        // can't have a negative number raised to a fractional power. In this situation need to
        // return max value
        if(rawOptical <= cParam) return maxDist;

        // compute the distance based on an inverse power law, i.e.
        //  distance = ((RawOptical - c)/a)^(1/b)
        double dist = Math.pow((rawOptical - cParam)/aParam, binvParam);

        // distance values change very rapidly with small values of RawOptical and become
        // impractical to fit to a distribution. Returns are capped to an experimentally determined
        // max value.
        return Math.min(dist, maxDist);
    }

    //----------------------------------------------------------------------------------------------
    // Raw sensor data
    //----------------------------------------------------------------------------------------------

    public int rawOptical()
    {
        // return raw value with overflow bit masked
        return (readUnsignedShort(Register.PS_DATA, ByteOrder.LITTLE_ENDIAN) & 0x7FF);
    }
}
