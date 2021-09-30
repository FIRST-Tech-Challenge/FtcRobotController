/*
 * Copyright (c) 2017 FTC team 4634 FROGbots
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

@I2cDeviceType
@DeviceProperties(name = "MaxSonar I2CXL", description = "MaxSonar I2CXL Ultrasonic Rangefinder from MaxBotix", xmlTag = "MaxSonarI2CXL")
public class MaxSonarI2CXL extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    private static final byte DEFAULT_I2C_ADDRESS = (byte) 0xE0;
    private static final byte CHANGE_I2C_ADDRESS_UNLOCK_1 = (byte) 0xAA;
    private static final byte CHANGE_I2C_ADDRESS_UNLOCK_2 = (byte) 0xA5;
    private final byte CMD_PING = 0x51;
    private final byte NUM_RANGE_BYTES = 2;
    private final int DEFAULT_SONAR_PROPAGATION_DELAY_MS = 50;

    private int lastDistance = -1;
    private long lastPingTime;

    private int numAttempts = 0;

    //------------------------------------------------------------------------------------------
    // Constructors
    //------------------------------------------------------------------------------------------

    public MaxSonarI2CXL(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(DEFAULT_I2C_ADDRESS));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected MaxSonarI2CXL(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned)
    {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    //------------------------------------------------------------------------------------------
    // I2cDeviceSync methods
    //------------------------------------------------------------------------------------------

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "MaxSonar I2CXL";
    }

    //------------------------------------------------------------------------------------------
    // Internal methods
    //------------------------------------------------------------------------------------------

    /***
     * Commands the sensor to send out a ping
     */
    private void ping()
    {
        deviceClient.write8(0, CMD_PING, I2cWaitControl.WRITTEN);
    }

    /***
     * Reads the range of the last commanded
     * measurement from the sensor
     *
     * @return the range of the last commanded
     *         measurement
     */
    private int getDistance()
    {
        int distance = TypeConversion.byteArrayToShort(deviceClient.read(0, NUM_RANGE_BYTES));

        if(distance != 0)
        {
            numAttempts = 0;
            return distance;
        }
        else
        {
            if(numAttempts >= 3)
            {
                return 0;
            }
            else
            {
                numAttempts++;

                System.out.println("GOT GARBAGE DATA FROM ULTRASONIC SENSOR");

                return getDistance();
            }
        }
    }

    //------------------------------------------------------------------------------------------
    // User sync methods
    //------------------------------------------------------------------------------------------

    /***
     * Commands the sensor to send out a ping,
     * sleeps for the default sonar propagation
     * delay, and then reads the result of the
     * measurement just commanded
     *
     * @return the result of the range measurement
     *         that was performed
     */
    public int getDistanceSync()
    {
        return getDistanceSync(DEFAULT_SONAR_PROPAGATION_DELAY_MS);
    }

    /***
     * Commands the sensor to send out a ping,
     * sleeps for the time specified by the
     * sonarPropagationDelay, and the reads the
     * result of the measurement just commanded
     *
     * @param sonarPropagationDelay controls how long to sleep after sending
     *                              out the ping before trying to read the
     *                              result of the measurement. If the value
     *                              is too short, then the sound wave might not
     *                              make it back to the sensor before the read
     *                              is performed. If it is too long, the loop
     *                              time of your program will be slowed without
     *                              purpose
     *
     * @return the result of the range measurement
     *         that was performed
     */
    public int getDistanceSync(int sonarPropagationDelay)
    {
        ping();
        try
        {
            sleep(sonarPropagationDelay);
        }
        catch (InterruptedException e)
        {
            currentThread().interrupt();
        }

        return getDistance();
    }

    //------------------------------------------------------------------------------------------
    // User async methods
    //------------------------------------------------------------------------------------------

    /***
     * Commands the sensor to send out a ping
     * if the default delay has expired, and
     * then reads the result of the measurement
     * just commanded. If the delay has NOT
     * expired, then the last value read from
     * the sensor will be returned instead.
     *
     * @return the result of the range measurement
     *         that was performed, or, if the default
     *         delay has not yet expired, the last
     *         value that was read from the sensor
     */
    public int getDistanceAsync()
    {
        return getDistanceAsync(DEFAULT_SONAR_PROPAGATION_DELAY_MS);
    }

    /***
     * Commands the sensor to send out a ping
     * if the delay specified has expired, and
     * then reads the result of the measurement
     * just commanded. If the delay has NOT
     * expired, then the last value read from
     * the sensor will be returned instead.
     *
     * @param sonarPropagationDelay controls how long to sleep after sending
     *                              out the ping before trying to read the
     *                              result of the measurement. If the value
     *                              is too short, then the sound wave might not
     *                              make it back to the sensor before the read
     *                              is performed. If it is too long, the sensor
     *                              will not report new data to your program
     *                              as fast as it could otherwise with a shorter
     *                              delay
     *
     * @return the result of the range measurement
     *         that was performed, or, if the delay
     *         specified by sonarPropagationDelay
     *         has not yet expired, the last value
     *         that was read from the sensor.
     */
    public int getDistanceAsync(int sonarPropagationDelay)
    {
        long curTime = System.currentTimeMillis();

        if(((curTime - lastPingTime) > sonarPropagationDelay))
        {
            lastDistance = getDistance();
            ping();
            lastPingTime = System.currentTimeMillis();
        }

        return lastDistance;
    }

    //------------------------------------------------------------------------------------------
    // User I2C address changing methods
    //------------------------------------------------------------------------------------------

    /***
     * Sets the I2C address the SDK should use when attempting
     * to communicate with the sensor. Note that this method
     * will NOT actually change the address inside the sensor,
     * for that see writeI2cAddrToSensorEEPROM(byte addr).
     *
     * If the aforementioned method is used to change the
     * address inside the sensor, then you will need to
     * call this method after getting the object from the
     * hardwareMap, and pass in the same address as you
     * wrote to the sensor's EEPROM.
     *
     * Note that you will need to call this method once at the
     * beginning of each of your opmodes after you get the object
     * from the hwMap, whereas you only need to call the method
     * that writes to the sensor's EEPROM, once, ever. As in you
     * only need to use that when you are initially wiring up the
     * sensor.
     *
     * @param i2cAddr the I2C address the SDK should use when
     *                attempting to communicate with the sensor
     */
    public void setI2cAddress(I2cAddr i2cAddr)
    {
        deviceClient.setI2cAddress(i2cAddr);
    }

    /***
     * Tells the sensor that it should operate on an I2C address
     * other than the default, and then writes that address to
     * the internal EEPROM so that it is retained across a power
     * cycle.
     *
     * You only need to call this method ONE TIME, EVER. (Unless
     * you want to change the address again, that is)
     *
     * Note that the sensor will only accept even numbered address
     * values. If an odd numbered address is sent, then the address
     * will be set to the next lowest even number. Also, the following
     * addresses are invalid, and if sent, the command will be ignored:
     *
     *      0x00
     *      0x50
     *      0xA4
     *      0xAA
     *
     * @param addr the I2C address the sensor should operate on
     */
    public void writeI2cAddrToSensorEEPROM(byte addr)
    {
        byte[] bytes = new byte[] {CHANGE_I2C_ADDRESS_UNLOCK_1, CHANGE_I2C_ADDRESS_UNLOCK_2, addr};
        deviceClient.write(0, bytes);
    }
}