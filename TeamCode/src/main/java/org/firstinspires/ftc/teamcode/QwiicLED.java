package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import android.graphics.Color;
import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType()
@DeviceProperties(name = "QWIIC LED Stick", description = "Sparkfun QWIIC LED Stick", xmlTag = "QWIIC_LED_STICK")
public class QwiicLED extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    
    private enum Commands {
        CHANGE_LED_LENGTH(0x70),
        WRITE_SINGLE_LED_COLOR(0x71),
        WRITE_ALL_LED_COLOR(0x72),
        WRITE_RED_ARRAY(0x73),
        WRITE_GREEN_ARRAY(0x74),
        WRITE_BLUE_ARRAY(0x75),
        WRITE_SINGLE_LED_BRIGHTNESS(0x76),
        WRITE_ALL_LED_BRIGHTNESS(0x77),
        WRITE_ALL_LED_OFF(0x78),
        WRITE_WALKING_LED (0x79),
        CHANGE_WRITE_DELAY (0x7a);
        
        int bVal;
        
        Commands(int bVal) {
            this.bVal = bVal;
        }
    }
    
    /**
     * Change the color of a specific LED
     *
     * @param position which LED to change (1 - 255)
     * @param color    what color to set it to
     */
    public void setColor(int position, @ColorInt int color) {
        byte[] data = new byte[4];
        data[0] = (byte) position;
        data[1] = (byte) Color.red(color);
        data[2] = (byte) Color.green(color);
        data[3] = (byte) Color.blue(color);
        writeI2C(Commands.WRITE_SINGLE_LED_COLOR, data);
    }
    
    /**
     * Change the color of all LEDs to a single color
     *
     * @param color what the color should be
     */
    public void setColor(@ColorInt int color) {
        byte[] data = new byte[3];
        data[0] = (byte) Color.red(color);
        data[1] = (byte) Color.green(color);
        data[2] = (byte) Color.blue(color);
        writeI2C(Commands.WRITE_ALL_LED_COLOR, data);
    }
    
    /**
     * Send a segment of the LED array
     *
     * @param cmd    command to send
     * @param array  the values (limited from 0..255)
     * @param offset the starting value (LED only, array starts at 0)
     * @param length the length to send
     */
    private void sendSegment(Commands cmd, int[] array, int offset, int length) {
        byte[] data = new byte[length + 2];
        data[0] = (byte) length;
        data[1] = (byte) offset;
        
        for (int i = 0; i < length; i++) {
            data[2 + i] = (byte) array[i];
        }
        writeI2C(cmd, data);
    }
    
    /**
     * Change the color of an LED color segment
     *
     * @param colors what the colors should be
     * @param offset where in the array to start
     * @param length length to send (limited to 12)
     */
    public void setLEDColorSegment(@ColorInt int[] colors, int offset, int length) {
        int[] redArray = new int[length];
        int[] greenArray = new int[length];
        int[] blueArray = new int[length];
        
        for (int i = 0; i < length; i++) {
            redArray[i] = Color.red(colors[i + offset]);
            greenArray[i] = Color.green(colors[i + offset]);
            blueArray[i] = Color.blue(colors[i + offset]);
        }
        sendSegment(Commands.WRITE_RED_ARRAY, redArray, offset, length);
        sendSegment(Commands.WRITE_GREEN_ARRAY, greenArray, offset, length);
        sendSegment(Commands.WRITE_BLUE_ARRAY, blueArray, offset, length);
    }
    
    /**
     * Change the color of an LED color segment
     *
     * @param color what the colors should be
     * @param offset where in the array to start
     * @param length length to send (limited to 12)
     */
    public void setLEDColorSegment(@ColorInt int color, int offset, int length) {
        int[] redArray = new int[length];
        int[] greenArray = new int[length];
        int[] blueArray = new int[length];
        
        for (int i = 0; i < length; i++) {
            redArray[i] = Color.red(color);
            greenArray[i] = Color.green(color);
            blueArray[i] = Color.blue(color);
        }
        sendSegment(Commands.WRITE_RED_ARRAY, redArray, offset, length);
        sendSegment(Commands.WRITE_GREEN_ARRAY, greenArray, offset, length);
        sendSegment(Commands.WRITE_BLUE_ARRAY, blueArray, offset, length);
    }
    
    /**
     * Change the color of all LEDs using arrays
     *
     * @param colors array of colors to set lights to
     */
    public void setColors(@ColorInt int[] colors) {
        int length = colors.length;
        
        int numInLastSegment = length % 12;
        int numSegments = length / 12;
        for (int i = 0; i < numSegments; i++) {
            setLEDColorSegment(colors, i * 12, 12);
        }
        setLEDColorSegment(colors, numSegments * 12, numInLastSegment);
    }
    
    /**
     * Shifts colors over once and inserts a new color
     *
     * @param color array of colors to set lights to
     * @param offset where in LED strip the walking lights should start
     * @param length length to shift over
     * @param direction the direction to shift colors (true = forwards)
     */
    public void setWalkingColor(@ColorInt int color, int brightness, int offset, int length, boolean direction) {
        byte[] data = new byte[7];
        data[0] = (byte) length;
        data[1] = (byte) offset;
        data[2] = (byte) (direction ? 1 : 0);
        data[3] = (byte) Color.red(color);
        data[4] = (byte) Color.green(color);
        data[5] = (byte) Color.blue(color);
        data[6] = (byte) brightness;
        writeI2C(Commands.WRITE_WALKING_LED, data);
    }
    
    /**
     * Set the brightness of an individual LED
     *
     * @param number     which LED to change (1-255)
     * @param brightness brightness level (0-31)
     */
    public void setBrightness(int number, int brightness) {
        byte[] data = new byte[2];
        data[0] = (byte) number;
        data[1] = (byte) brightness;
        writeI2C(Commands.WRITE_SINGLE_LED_BRIGHTNESS, data);
    }
    
    /**
     * Set the brightness of all LEDs
     *
     * @param brightness brightness level (0-31)
     */
    public void setBrightness(int brightness) {
        byte[] data = new byte[1];
        data[0] = (byte) brightness;
        writeI2C(Commands.WRITE_ALL_LED_BRIGHTNESS, data);
    }
    
    /**
     * Turn all LEDS off...
     */
    public void turnAllOff() {
        setColor(0);
    }
    
    /**
     * Change the length of the LED strip
     *
     * @param newLength 1 to 100 (longer than 100 not supported)
     */
    public void changeLength(int newLength) {
        byte[] data = new byte[1];
        data[0] = (byte) newLength;
        writeI2C(Commands.CHANGE_LED_LENGTH, data);
    }
    
    /**
     * Change the delay after receiving i2c data when the Qwiic writes to the LEDs
     *
     * @param delay 1 to 255 (in tenths of a millisecond)
     */
    public void changeWriteDelay(int delay) {
        byte[] data = new byte[1];
        data[0] = (byte) delay;
        writeI2C(Commands.CHANGE_WRITE_DELAY, data);
    }
    
    private void writeI2C(Commands cmd, byte[] data) {
        deviceClient.write(cmd.bVal, data, I2cWaitControl.WRITTEN);
    }
    
    
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    
    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }
    
    @Override
    public String getDeviceName() {
        return "Qwiic LED Strip";
    }
    
    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x23);
    
    public QwiicLED(I2cDeviceSynchSimple deviceClient) {
        super(deviceClient, true);
        
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
    }
    
}