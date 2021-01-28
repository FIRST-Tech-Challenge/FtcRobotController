/*
 * Copyright (c) 2018 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.test.revextensions2;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxCommExceptionHandler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxPhoneChargeControlCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxPhoneChargeQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxPhoneChargeQueryResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusResponse;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Provides an implementation of the REV Expansion Hub's features that are not
 * included in the standard FTC SDK.
 */
public class ExpansionHubEx extends LynxCommExceptionHandler implements HardwareDevice
{
    private LynxModule expansionHub;

    /*
     * Don't use this constructor in user-code; this object will be
     * hotswapped into the hardwareMap at runtime.
     */
    ExpansionHubEx(LynxModule expansionHub)
    {
        this.expansionHub = expansionHub;
    }

    public LynxModule getStandardModule()
    {
        return expansionHub;
    }

    /***
     * Set the color of the Expansion Hub's RGB status LED
     *
     * @param r red value
     * @param g green value
     * @param b blue value
     */
    public synchronized void setLedColor(int r, int g, int b)
    {
        if(r > 255 || g > 255 || b > 255)
        {
            throw new IllegalArgumentException();
        }

        setLedColor((byte)r, (byte)g, (byte)b);
    }

    /***
     * Set the color of the Expansion Hub's RGB status LED
     *
     * @param r red value
     * @param g green value
     * @param b blue value
     */
    public synchronized void setLedColor(byte r, byte g, byte b)
    {
        LynxSetModuleLEDColorCommand colorCommand = new LynxSetModuleLEDColorCommand(expansionHub, r, g, b);
        try
        {
            colorCommand.send();
        }
        catch (InterruptedException | LynxNackException e)
        {
            handleException(e);
        }
    }

    /***
     * Set the color of the Expansion Hub's RGB status LED
     *
     * @param resId the resId of a color you have defined in your colors.xml file
     */
    public synchronized void setLedColor(int resId)
    {
        int color = AppUtil.getInstance().getRootActivity().getResources().getColor(resId);

        byte red = (byte) Color.red(color);
        byte green = (byte) Color.green(color);
        byte blue = (byte) Color.blue(color);

        setLedColor(red, green, blue);
    }

    public enum I2cBusSpeed
    {
        STANDARD_100K,
        FAST_400K,
        FASTPLUS_1M,
        HIGH_3_4M
    }

    /***
     * Set the speed of all of the I2C buses on the Expansion Hub
     * in one go!
     *
     * @param speed the speed to set all of the buses to
     */
    public synchronized void setAllI2cBusSpeeds(I2cBusSpeed speed)
    {
        for(int i = 0; i < LynxConstants.NUMBER_OF_I2C_BUSSES; i++)
        {
            setI2cBusSpeed(i, speed);
        }
    }

    /***
     * Set the speed of an individual I2C bus on the Expansion Hub
     *
     * @param bus the bus to set the speed of (0...3)
     * @param speed the speed to set the bus to
     */
    public synchronized void setI2cBusSpeed(int bus, I2cBusSpeed speed)
    {
        LynxI2cConfigureChannelCommand.SpeedCode speedCode = LynxI2cConfigureChannelCommand.SpeedCode.STANDARD_100K;

        switch (speed)
        {
            case STANDARD_100K:
                speedCode = LynxI2cConfigureChannelCommand.SpeedCode.STANDARD_100K;
                break;

            case FAST_400K:
                speedCode = LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K;
                break;

            case FASTPLUS_1M:
                speedCode = LynxI2cConfigureChannelCommand.SpeedCode.FASTPLUS_1M;
                break;

            case HIGH_3_4M:
                speedCode = LynxI2cConfigureChannelCommand.SpeedCode.HIGH_3_4M;
                break;
        }

        LynxI2cConfigureChannelCommand command = new LynxI2cConfigureChannelCommand(expansionHub, bus, speedCode);
        try
        {
            command.send();
        }
        catch (LynxNackException | InterruptedException e)
        {
            handleException(e);
        }
    }

    public enum CurrentDrawUnits
    {
        MILLIAMPS,
        AMPS
    }

    /**
     * Get the total current draw for the entire Expansion Hub.
     *
     * @param units the units to return the current draw in
     * @return Current draw for the entire Expansion Hub
     */
    public synchronized double getTotalModuleCurrentDraw(CurrentDrawUnits units)
    {
        LynxGetADCCommand command = new LynxGetADCCommand(expansionHub, LynxGetADCCommand.Channel.BATTERY_CURRENT, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            LynxGetADCResponse response = command.sendReceive();

            int ma = response.getValue();

            if(units == CurrentDrawUnits.MILLIAMPS)
            {
                return ma;
            }
            else if(units == CurrentDrawUnits.AMPS)
            {
                return ma/1000d;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
            handleException(e);
        }
        return 0;
    }

    /**
     * Get the current draw for the servo bus.
     *
     * @param units the units to return the current draw in
     * @return Current draw of the servo bus
     * @deprecated This feature currently does not work (likely a bug in the Expansion Hub firmware).
     */
    @Deprecated
    public synchronized double getServoBusCurrentDraw(CurrentDrawUnits units)
    {
        LynxGetADCCommand.Channel channel = LynxGetADCCommand.Channel.SERVO_CURRENT;

        LynxGetADCCommand command = new LynxGetADCCommand(expansionHub, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            LynxGetADCResponse response = command.sendReceive();

            int ma = response.getValue();

            if(units == CurrentDrawUnits.MILLIAMPS)
            {
                return ma;
            }
            else if(units == CurrentDrawUnits.AMPS)
            {
                return ma/1000d;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
            handleException(e);
        }
        return 0;
    }

    /***
     * Get the total current draw for the GPIO bus
     *
     * @param units the units to return the current draw in
     * @return current draw of the GPIO bus
     */
    public synchronized double getGpioBusCurrentDraw(CurrentDrawUnits units)
    {
        LynxGetADCCommand.Channel channel = LynxGetADCCommand.Channel.GPIO_CURRENT;

        LynxGetADCCommand command = new LynxGetADCCommand(expansionHub, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            LynxGetADCResponse response = command.sendReceive();

            int ma = response.getValue();

            if(units == CurrentDrawUnits.MILLIAMPS)
            {
                return ma;
            }
            else if(units == CurrentDrawUnits.AMPS)
            {
                return ma/1000d;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
            handleException(e);
        }
        return 0;
    }

    /***
     * Get the total current draw for the I2C bus
     *
     * @param units the units to return the current draw in
     * @return the current draw of the I2C bus
     */
    public synchronized double getI2cBusCurrentDraw(CurrentDrawUnits units)
    {
        LynxGetADCCommand.Channel channel = LynxGetADCCommand.Channel.I2C_BUS_CURRENT;

        LynxGetADCCommand command = new LynxGetADCCommand(expansionHub, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            LynxGetADCResponse response = command.sendReceive();

            int ma = response.getValue();

            if(units == CurrentDrawUnits.MILLIAMPS)
            {
                return ma;
            }
            else if(units == CurrentDrawUnits.AMPS)
            {
                return ma/1000d;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
            handleException(e);
        }
        return 0;
    }

    public enum VoltageUnits
    {
        MILLIVOLTS,
        VOLTS
    }

    /***
     * Get the voltage reported by the Hub's 5v rail monitor
     *
     * @param units the units to return the voltage in
     * @return the voltage reported by the Hub's 5v monitor
     */
    public synchronized double read5vMonitor(VoltageUnits units)
    {
        LynxGetADCCommand.Channel channel = LynxGetADCCommand.Channel.FIVE_VOLT_MONITOR;

        LynxGetADCCommand command = new LynxGetADCCommand(expansionHub, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            LynxGetADCResponse response = command.sendReceive();

            int mv = response.getValue();

            if(units == VoltageUnits.MILLIVOLTS)
            {
                return mv;
            }
            else if(units == VoltageUnits.VOLTS)
            {
                return mv/1000d;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
            handleException(e);
        }
        return 0;
    }

    /***
     * Get the voltage reported by the Hub's 12v rail monitor
     *
     * @param units the units to return the voltage in
     * @return the voltage reported by the Hub's 12v monitor
     */
    public synchronized double read12vMonitor(VoltageUnits units)
    {
        LynxGetADCCommand.Channel channel = LynxGetADCCommand.Channel.BATTERY_MONITOR;

        LynxGetADCCommand command = new LynxGetADCCommand(expansionHub, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            LynxGetADCResponse response = command.sendReceive();

            int mv = response.getValue();

            if(units == VoltageUnits.MILLIVOLTS)
            {
                return mv;
            }
            else if(units == VoltageUnits.VOLTS)
            {
                return mv/1000d;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
            handleException(e);
        }
        return 0;
    }

    public enum TemperatureUnits
    {
        CELSIUS,
        FAHRENHEIT
    }

    /***
     * Get the temperature reported by the Hub's internal temperature sensor
     *
     * @param units the units to return the temperature in
     * @return the Hub's internal temperature
     */
    public synchronized double getInternalTemperature(TemperatureUnits units)
    {
        LynxGetADCCommand.Channel channel = LynxGetADCCommand.Channel.CONTROLLER_TEMPERATURE;

        LynxGetADCCommand command = new LynxGetADCCommand(expansionHub, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            LynxGetADCResponse response = command.sendReceive();

            int degC = response.getValue();

            if(units == TemperatureUnits.CELSIUS)
            {
                return degC;
            }
            else if(units == TemperatureUnits.FAHRENHEIT)
            {
                return (degC*(9d/5d)) + 32;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
            handleException(e);
        }
        return 0;
    }

    /***
     * Query if the Lynx Module is in an over-temp condition
     *
     * @return boolean indicating if the module is over-temp
     */
    public synchronized boolean isModuleOverTemp()
    {
        LynxGetModuleStatusCommand command = new LynxGetModuleStatusCommand(expansionHub);

        try
        {
            LynxGetModuleStatusResponse response = command.sendReceive();
            return response.isControllerOverTemp();
        }

        catch (Exception e)
        {
            handleException(e);
        }

        return false;
    }

    /***
     * Query if the Lynx Module has the phone charge feature enabled
     * @deprecated because this seems to lock up the Hub internally and crash the SDK
     *
     * @return boolean indicating if the module has the phone charge feature enabled
     */
    @Deprecated
    public synchronized boolean isPhoneChargeEnabled()
    {
        LynxPhoneChargeQueryCommand command = new LynxPhoneChargeQueryCommand(expansionHub);

        try
        {
            LynxPhoneChargeQueryResponse response = command.sendReceive();
            return response.isChargeEnabled();
        }

        catch (Exception e)
        {
            handleException(e);
        }

        return false;
    }

    /***
     * Control whether or not the Lynx Module's phone charging function is enabled
     *
     * @param chargeEnabled true for enabled, false for disabled
     */
    public synchronized void setPhoneChargeEnabled(boolean chargeEnabled)
    {
        LynxPhoneChargeControlCommand controlCommand = new LynxPhoneChargeControlCommand(expansionHub, chargeEnabled);
        try
        {
            controlCommand.send();
        }
        catch (InterruptedException | LynxNackException e)
        {
            handleException(e);
        }
    }

    /***
     * Set the pulse width output of a servo port directly, rather than using
     * the -1 to 1 range from the SDK
     *
     * @param port the servo port to set the pulse width of
     * @param uS the pulse width (in uS) to set the above servo port to
     */
    public synchronized void setServoPulseWidth(int port, int uS)
    {
        LynxSetServoPulseWidthCommand command = new LynxSetServoPulseWidthCommand(expansionHub, port, uS);

        try
        {
            command.send();
        }
        catch (InterruptedException | LynxNackException e)
        {
            handleException(e);
        }
    }

    /***
     * Grab a bunch of useful data from the Lynx module in one go
     *
     * @return an object that contains a bunch of useful data
     */
    public synchronized RevBulkData getBulkInputData()
    {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(expansionHub);

        try
        {
            LynxGetBulkInputDataResponse response = command.sendReceive();
            return new RevBulkData(response, expansionHub);
        }

        catch (Exception e)
        {
            handleException(e);
        }

        return null;
    }

    /**
     * Get the amount of current that a given motor is pulling from its H-bridge
     *
     * @param units the units to return the current draw in
     * @param port the port of the motor in question
     * @return the current draw in milliamps
     */
    public synchronized double getMotorCurrentDraw(CurrentDrawUnits units, int port)
    {
        LynxConstants.validateMotorZ(port);

        LynxGetADCCommand.Channel channel = null;

        if (port == 0)
        {
            channel = LynxGetADCCommand.Channel.MOTOR0_CURRENT;
        }
        else if (port == 1)
        {
            channel = LynxGetADCCommand.Channel.MOTOR1_CURRENT;
        }
        else if (port == 2)
        {
            channel = LynxGetADCCommand.Channel.MOTOR2_CURRENT;
        }
        else if (port == 3)
        {
            channel = LynxGetADCCommand.Channel.MOTOR3_CURRENT;
        }

        LynxGetADCCommand command = new LynxGetADCCommand(expansionHub, channel, LynxGetADCCommand.Mode.ENGINEERING);

        try
        {
            LynxGetADCResponse response = command.sendReceive();

            int ma = response.getValue();

            if(units == CurrentDrawUnits.MILLIAMPS)
            {
                return ma;
            }
            else if(units == CurrentDrawUnits.AMPS)
            {
                return ma/1000d;
            }
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
            handleException(e);
        }

        return 0;
    }

    /***
     * Query as to whether the H-bridge for a given motor port is over-temp
     *
     * @param port the motor port in question
     * @return boolean indicating the H-bridge is over-temp
     */
    public synchronized boolean isMotorBridgeOverTemp(int port)
    {
        LynxGetModuleStatusCommand command = new LynxGetModuleStatusCommand(expansionHub);

        try
        {
            LynxGetModuleStatusResponse response = command.sendReceive();
            return response.isMotorBridgeOverTemp(port);
        }

        catch (Exception e)
        {
            handleException(e);
        }

        return false;
    }

    /***
     * Query as to whether a given motor has lost (encoder?) counts
     * @deprecated because I have no idea what this actually does
     *
     * @param port the port of the motor in question
     * @return boolean indicating whether the motor has lost (encoder?) counts
     */
    @Deprecated
    public synchronized boolean hasMotorLostCounts(int port)
    {
        LynxGetModuleStatusCommand command = new LynxGetModuleStatusCommand(expansionHub);

        try
        {
            LynxGetModuleStatusResponse response = command.sendReceive();
            return response.hasMotorLostCounts(port);
        }

        catch (Exception e)
        {
            e.printStackTrace();
        }

        return false;
    }

    /***
     * Query as to which firmware this Expansion Hub is running
     *
     * @return firmware version string, in th format of maj.min.eng
     */
    public synchronized String getFirmwareVersion()
    {
        //HW: 20, Maj: 1, Min: 7, Eng: 0

        String inputstring = expansionHub.getFirmwareVersionString();

        if(inputstring.equals("unknown firmware") || inputstring.equals("firmware version unavailable"))
        {
            return inputstring;
        }

        int majVer = regexField("Maj", inputstring);
        int minVer = regexField("Min", inputstring);
        int engVer = regexField("Eng", inputstring);

        return String.format("%s.%s.%s", majVer, minVer, engVer);
    }

    /***
     * Ask the Expansion Hub which hardware revision it is
     *
     * @return the hardware revision
     */
    public synchronized int getHardwareRevision()
    {
        //HW: 20, Maj: 1, Min: 7, Eng: 0

        String inputstring = expansionHub.getFirmwareVersionString();

        if(inputstring.equals("unknown firmware") || inputstring.equals("firmware version unavailable"))
        {
            return -1;
        }

        return regexField("HW", inputstring);
    }

    private int regexField(String thing, String input)
    {
        Pattern pattern = Pattern.compile(thing + ":\\ \\d+");
        Matcher matcher = pattern.matcher(input);
        matcher.find();

        String string = matcher.group();

        return regexNum(string);
    }

    private int regexNum(String str)
    {
        Pattern pattern = Pattern.compile("\\d+");
        Matcher matcher = pattern.matcher(str);
        matcher.find();

        return Integer.parseInt(matcher.group());
    }

    //-----------------------------------------------------------------------------------------
    // Ignore the following methods....
    //
    // They're only here because in order to have this object hot-injected into the hardwareMap,
    // it needs to implement HardwareDevice. And well, in order to implement HardwareDevice, we
    // need to implement these methods. Sooo yeah...
    //-----------------------------------------------------------------------------------------

    @Override
    public Manufacturer getManufacturer()
    {
        return expansionHub.getManufacturer();
    }

    @Override
    public String getDeviceName()
    {
        return expansionHub.getDeviceName();
    }

    @Override
    public String getConnectionInfo()
    {
        return expansionHub.getConnectionInfo();
    }

    @Override
    public int getVersion()
    {
        return expansionHub.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode()
    {
        expansionHub.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close()
    {
        expansionHub.close();
    }
}