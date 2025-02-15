// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode;

public class Utils {
  public static double getTimeSeconds() {
    return System.nanoTime() / 1e9;
  }

  public static double getTimeMiliSeconds() {
    return System.nanoTime() / 1e6;
  }


  /**
   * Map one range of numbers to another
   *
   * @param input - Input variable
   * @param old_Lower - Lower range of input
   * @param old_Upper - Upper range of input
   * @param new_Lower - Lower range of output
   * @param new_Upper - Upper range of output
   * @return - New mapped variable
   *
   * @see #map(double, double, double, double, double)
   * @see #map(int, int, int, int, int)
   */
  public static double map(double input, double old_Lower, double old_Upper, double new_Lower, double new_Upper) {
    return (input - old_Lower) / (old_Upper - old_Lower) * (new_Upper - new_Lower) + new_Lower;
  }

  /**
   * Map one range of numbers to another
   *
   * @param input - Input variable
   * @param old_Lower - Lower range of input
   * @param old_Upper - Upper range of input
   * @param new_Lower - Lower range of output
   * @param new_Upper - Upper range of output
   * @return - New mapped variable
   *
   * @see #map(double, double, double, double, double)
   * @see #map(int, int, int, int, int)
   */
  public static int map(int input, int old_Lower, int old_Upper, int new_Lower, int new_Upper) {
    return (input - old_Lower) / (old_Upper - old_Lower) * (new_Upper - new_Lower) + new_Lower;
  }
}
