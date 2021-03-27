package com.technototes.library.hardware.sensor;

import android.graphics.Color;

/** Class for color sensors
 * @author Alex Stedman
 */
public class ColorSensor extends Sensor<com.qualcomm.robotcore.hardware.ColorSensor> {
    /** Make a color Sensor
     *
     * @param device The hardware device
     */
    public ColorSensor(com.qualcomm.robotcore.hardware.ColorSensor device) {
        super(device);
    }

    /** Make a color sensor
     *
     * @param deviceName The device name in hardware map
     */
    public ColorSensor(String deviceName) {
        super(deviceName);
    }

    @Override
    public double getSensorValue() {
        return rgb();
    }

    /** Get the RGB of the sensor
     *
     * @return RGB int in format REDGRNBLU
     */
    public int rgb(){
        return getDevice().argb() >> 3;
    }
    /** Get the RGB red of the sensor
     *
     * @return Red
     */
    public int red(){
        return getDevice().red();
    }
    /** Get the RGB green of the sensor
     *
     * @return Green
     */
    public int green(){
        return getDevice().green();
    }
    /** Get the RGB blue of the sensor
     *
     * @return Blue
     */
    public int blue(){
        return getDevice().blue();
    }

    /** Get the alpha (transparency) of the color
     *
     * @return Alpha
     */
    public int alpha(){
        return getDevice().alpha();
    }

    /** Get HSV as an int
     *
     * @return HSV
     */
    public int hsv(){
        return (hue() << 16) | (saturation() << 8) | value();
    }

    private float[] hsvAsArray(){
        float[] f = new float[3];
        Color.RGBToHSV(red(),green(),blue(),f);
        return f;
    }

    /** Get HSV hue
     *
     * @return Hue
     */
    public int hue(){
        return (int)hsvAsArray()[0];
    }

    /** Get HSV saturation
     *
     * @return Saturation
     */
    public int saturation(){
        return (int)hsvAsArray()[1];
    }

    /** Get HSV value (not entire HSV, just 'V')
     *
     * @return Value
     */
    public int value(){
        return (int)hsvAsArray()[2];
    }
}
