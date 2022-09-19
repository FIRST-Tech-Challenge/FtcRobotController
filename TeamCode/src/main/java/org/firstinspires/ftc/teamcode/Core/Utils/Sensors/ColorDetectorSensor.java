package org.firstinspires.ftc.teamcode.Core.Utils.Sensors;

import android.graphics.Color;

import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorDetectorSensor implements HardwareDevice {

  private final ColorSensor colorSensor;

  /** Constructs a color sensor, defaults to ARGB */
  public ColorDetectorSensor(ColorSensor colorSensor) {
    this.colorSensor = colorSensor;
  }

  /** Constructs a color sensor using the given hardware map and name, defaults to ARGB */
  public ColorDetectorSensor(HardwareMap hardwareMap, String name) {
    this(hardwareMap.get(ColorSensor.class, name));
  }

  /**
   * Convert HSV value to an ARGB one. Includes alpha.
   *
   * @return an int representing the ARGB values
   */
  public int[] HSVtoARGB(int alpha, float[] hsv) {
    int color = Color.HSVToColor(alpha, hsv);
    return new int[] {Color.alpha(color), Color.red(color), Color.green(color), Color.blue(color)};
  }

  /** Converts an RGB value to an HSV value. Provide the float[] to be used. */
  public float[] RGBtoHSV(int red, int green, int blue, float[] hsv) {
    Color.RGBToHSV(red, green, blue, hsv);
    return hsv;
  }

  /**
   * Get all the ARGB values in an array from the sensor
   *
   * @return an int array representing ARGB
   */
  public int[] getARGB() {
    return new int[] {alpha(), red(), green(), blue()};
  }

  /** Gets the alpha value from the sensor */
  public int alpha() {
    return colorSensor.alpha();
  }

  /** Gets the red value from the sensor */
  public int red() {
    return colorSensor.red();
  }

  /** Gets the green value from the sensor */
  public int green() {
    return colorSensor.green();
  }

  /** Gets the blue value from the sensor */
  public int blue() {
    return colorSensor.blue();
  }

  @Override
  public void disable() {
    colorSensor.close();
  }

  @Override
  public String getDeviceType() {
    return "Color Sensor";
  }
}
