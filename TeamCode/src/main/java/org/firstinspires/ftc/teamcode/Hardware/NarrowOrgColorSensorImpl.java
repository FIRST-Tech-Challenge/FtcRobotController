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

import android.graphics.Color;
import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;

/**
 * BroadcomColorSensorImpl is used to support the Rev Robotics V3 color sensor
 */
@SuppressWarnings("WeakerAccess")
public abstract class NarrowOrgColorSensorImpl extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynchSimple, BroadcomColorSensor.Parameters>
        implements BroadcomColorSensor, I2cAddrConfig, Light
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public static final String TAG = "BroadcomColorSensorImpl";
    NormalizedRGBA colors = new NormalizedRGBA();
    int red = 0, green = 0, blue = 0, alpha = 0;
    float softwareGain = 1;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    protected NarrowOrgColorSensorImpl(BroadcomColorSensor.Parameters params,
                                      I2cDeviceSynchSimple deviceClient,
                                      boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned, params);
        this.deviceClient.setLogging(this.parameters.loggingEnabled);
        this.deviceClient.setLoggingTag(this.parameters.loggingTag);

        // We ask for an initial call back here; that will eventually call internalInitialize()
        this.registerArmingStateCallback(true);

        this.engage();
    }

    //----------------------------------------------------------------------------------------------
    // Initialization
    //----------------------------------------------------------------------------------------------

    @Override
    protected synchronized boolean internalInitialize(@NonNull Parameters parameters)
    {
        RobotLog.vv(TAG, "internalInitialize()...");
        try {
            if (this.parameters.deviceId != parameters.deviceId)
            {
                throw new IllegalArgumentException(String.format("can't change device types (modify existing params instead): old=%d new=%d", this.parameters.deviceId, parameters.deviceId));
            }

            // Remember the parameters for future use
            this.parameters = parameters.clone();

            // Make sure we're talking to the correct I2c device
            this.setI2cAddress(parameters.i2cAddr);

            // Can't do anything if we're not really talking to the hardware
            if (!this.deviceClient.isArmed())
            {
                return false;
            }

            // Verify that that's a color sensor!
            byte id = this.getDeviceID();
            if (id != parameters.deviceId)
            {
                RobotLog.ee(TAG, "unexpected Broadcom color sensor chipid: found=%d expected=%d", id, parameters.deviceId);
                return false;
            }

            // sanity check: before
            dumpState();

            // set the gain, LED parameters and resolution
            setHardwareGain(parameters.gain);
            setLEDParameters(parameters.pulseModulation, parameters.ledCurrent);
            setProximityPulseCount(parameters.proximityPulseCount);
            setPSRateAndRes(parameters.proximityResolution, parameters.proximityMeasRate);
            setLSRateAndRes(parameters.lightSensorResolution, parameters.lightSensorMeasRate);

            // Enable the device
            enable();

            // sanity check: after
            dumpState();

            // Set up a read-ahead, if supported and requested
            if (this.deviceClient instanceof I2cDeviceSynch && parameters.readWindow != null)
            {
                I2cDeviceSynch synch = ((I2cDeviceSynch)this.deviceClient);
                synch.setReadWindow(parameters.readWindow);
            }

            return true;
        }
        finally
        {
            RobotLog.vv(TAG, "...internalInitialize()");
        }
    }

    protected void dumpState()
    {
        int cb = 0x07;
        RobotLog.logBytes(TAG, "state", read(Register.MAIN_CTRL, cb), cb);
    }

    protected synchronized void enable()
    {
        byte enabled = readMainCtrl();
        RobotLog.vv(TAG, "enable() enabled=0x%02x...", enabled);

        // enable ambient light sensor, ALS/IR channels, and proximity
        write8(Register.MAIN_CTRL, MainControl.PS_EN.bVal | MainControl.LS_EN.bVal | MainControl.RGB_MODE.bVal);

        enabled = readMainCtrl();
        RobotLog.vv(TAG, "...enable() enabled=0x%02x", enabled);

        return;
    }

    protected synchronized void disable()
    {
        byte enabled = readMainCtrl();
        RobotLog.vv(TAG, "disable() enabled=0x%02x...", enabled);

        write8(Register.MAIN_CTRL, enabled & ~(MainControl.PS_EN.bVal | MainControl.LS_EN.bVal | MainControl.RGB_MODE.bVal));
        enabled = readMainCtrl();

        RobotLog.vv(TAG, "...disable() enabled=0x%02x", enabled);
    }

    protected boolean testBits(byte value, byte desired)
    {
        return testBits(value, desired, desired);
    }
    protected boolean testBits(byte value, byte mask, byte desired)
    {
        return (value & mask) == desired;
    }

    protected byte readMainCtrl()
    {
        return read8(Register.MAIN_CTRL);
    }

    protected void setProximityPulseCount(int proximityPulseCount)
    {
        RobotLog.vv(TAG, "setProximityPulseCount(0x%02x)", proximityPulseCount);
        write8(Register.PS_PULSES, proximityPulseCount);
    }

    protected void setHardwareGain(Gain gain)
    {
        RobotLog.vv(TAG, "setGain(0x%02x)", gain.bVal);
        write8(Register.LS_GAIN, gain.bVal);
    }

    protected void setPDrive(LEDCurrent ledDrive)
    {
        RobotLog.vv(TAG, "setPDrive(0x%02x)", ledDrive.bVal);
        write8(Register.PS_LED, ledDrive.bVal);
    }

    public byte getDeviceID()
    {
        return this.read8(Register.PART_ID);
    }

    protected  void setLEDParameters(LEDPulseModulation pulseMod, LEDCurrent curr)
    {
        int val = (pulseMod.bVal << 4) | curr.bVal;
        RobotLog.vv(TAG, "setLEDParameters(0x%02x)", (byte) val);
        write8(Register.PS_LED, (byte) val);
    }

    protected void setPSRateAndRes(PSResolution res, PSMeasurementRate rate)
    {
        int val = (res.bVal << 3) | rate.bVal;
        RobotLog.vv(TAG, "setPSMeasRate(0x%02x)", (byte) val);
        write8(Register.PS_MEAS_RATE, (byte) val);
    }

    protected void setLSRateAndRes(LSResolution res, LSMeasurementRate rate)
    {
        int val = (res.bVal << 4) | rate.bVal;
        RobotLog.vv(TAG, "setLSMeasRate(0x%02x)", (byte) val);
        write8(Register.LS_MEAS_RATE, (byte) val);
    }

    //----------------------------------------------------------------------------------------------
    // Interfaces
    //----------------------------------------------------------------------------------------------

    /** In this implementation, the {@link Color} methods return 16 bit unsigned values. */

    @Override
    public synchronized int red() { return this.red; }

    @Override
    public synchronized int green() { return this.green; }

    @Override
    public synchronized int blue() { return this.blue; }

    @Override
    public synchronized int alpha() { return this.alpha; }

    @Override
    public synchronized @ColorInt int argb() { return getNormalizedColors().toColor(); }

    @Override
    public void setGain(float newGain) { this.softwareGain = newGain; }

    @Override
    public float getGain() { return this.softwareGain; }

    public void updateColors()
    {
        // Check for updated color values. If new data is not ready yet, keep the last read values
        byte mainStatus;
        byte[] data = null;

        // check data status
        mainStatus = read8(Register.MAIN_STATUS);

        // update color values if LS_DATA is valid
        if (testBits(mainStatus, MainStatus.LS_DATA_STATUS.bVal))
        {
            // Read red, green and blue values
            final int cbRead = 9;
            data = read(Register.LS_DATA_GREEN, cbRead);

            final int dib = 0;
            this.green = TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, dib, ByteOrder.LITTLE_ENDIAN));
            this.blue = Range.clip((int) (1.55 * TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(
                    data, dib + 3, ByteOrder.LITTLE_ENDIAN))), 0, 65535);
            this.red = Range.clip((int) (1.07 * TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(
                    data, dib + 6, ByteOrder.LITTLE_ENDIAN))), 0, 65535);

            this.alpha = (this.red + this.green + this.blue) / 3;

            // normalize to [0, 1]
            this.colors.red = Range.clip(((float)this.red * this.softwareGain) / parameters.colorSaturation, 0f, 1f);
            this.colors.green = Range.clip(((float)this.green * this.softwareGain) / parameters.colorSaturation, 0f, 1f);
            this.colors.blue = Range.clip(((float)this.blue * this.softwareGain) / parameters.colorSaturation, 0f, 1f);

            // apply inverse squared law of light to get readable brightness value, stored in alpha channel
            // scale to 65535
            float avg = (float)(this.red + this.green + this.blue) / 3;
            this.colors.alpha = (float) (-(65535f / (Math.pow(avg, 2) + 65535)) + 1);
        }
    }

    @Override
    public NormalizedRGBA getNormalizedColors() { return this.colors; }

    @Override
    public synchronized void enableLed(boolean enable)
    {
        // We can't directly control the LED with I2C; it's always on
        // ignore; used to throw an error, but the default OpMode can try to turn off
        // (for range sensor variant) so got constant exceptions
    }

    @Override public boolean isLightOn()
    {
        return true;
    }

    @Override
    public synchronized I2cAddr getI2cAddress()
    {
        return this.deviceClient.getI2cAddress();
    }

    @Override
    public synchronized void setI2cAddress(I2cAddr i2cAddr)
    {
        this.parameters.i2cAddr = i2cAddr;
        this.deviceClient.setI2cAddress(i2cAddr);
    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    @Override
    public String getDeviceName() { return "Broadcom I2C Color Sensor"; }

    @Override
    public Manufacturer getManufacturer() { return Manufacturer.Broadcom; }

    @Override
    public void resetDeviceConfigurationForOpMode()
    {
        super.resetDeviceConfigurationForOpMode();
        this.softwareGain = 1;
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    protected int readUnsignedByte(final Register reg)
    {
        return TypeConversion.unsignedByteToInt(read8(reg));
    }

    protected int readUnsignedShort(final Register reg, ByteOrder byteOrder)
    {
        byte[] data = read(reg, 2);
        return TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, 0, byteOrder));
    }

    @Override
    public synchronized byte read8(final Register reg)
    {
        return deviceClient.read8(reg.bVal);
    }

    @Override
    public synchronized byte[] read(final Register reg, final int cb)
    {
        return deviceClient.read(reg.bVal, cb);
    }

    @Override
    public synchronized void write8(Register reg, int data)
    {
        this.deviceClient.write8(reg.bVal, data, I2cWaitControl.WRITTEN);
    }

    @Override public void write(Register reg, byte[] data)
    {
        this.deviceClient.write(reg.bVal, data, I2cWaitControl.WRITTEN);
    }

    protected void delay(int ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
    }
}
