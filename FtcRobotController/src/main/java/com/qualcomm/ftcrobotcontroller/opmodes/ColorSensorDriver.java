/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ColorSensorDriver extends LinearOpMode {

  public enum ColorSensorDevice {ADAFRUIT, HITECHNIC_NXT, MODERN_ROBOTICS_I2C};

  public ColorSensorDevice device = ColorSensorDevice.MODERN_ROBOTICS_I2C;

  ColorSensor colorSensor;
  DeviceInterfaceModule cdim;
  LED led;
  TouchSensor t;

  @Override
  public void runOpMode() throws InterruptedException {
    hardwareMap.logDevices();

    cdim = hardwareMap.deviceInterfaceModule.get("dim");
    switch (device) {
      case HITECHNIC_NXT:
        colorSensor = hardwareMap.colorSensor.get("nxt");
        break;
      case ADAFRUIT:
        colorSensor = hardwareMap.colorSensor.get("lady");
        break;
      case MODERN_ROBOTICS_I2C:
        colorSensor = hardwareMap.colorSensor.get("mr");
        break;
    }
    led = hardwareMap.led.get("led");
    t = hardwareMap.touchSensor.get("t");

    waitForStart();

    float hsvValues[] = {0,0,0};
    final float values[] = hsvValues;
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
    while (opModeIsActive()) {

      enableLed(t.isPressed());

      switch (device) {
        case HITECHNIC_NXT:
          Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
          break;
        case ADAFRUIT:
          Color.RGBToHSV((colorSensor.red() * 255) / 800, (colorSensor.green() * 255) / 800, (colorSensor.blue() * 255) / 800, hsvValues);
          break;
        case MODERN_ROBOTICS_I2C:
          Color.RGBToHSV(colorSensor.red()*8, colorSensor.green()*8, colorSensor.blue()*8, hsvValues);
          break;
      }
      telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.addData("Hue", hsvValues[0]);

      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });
      waitOneFullHardwareCycle();
    }
  }

  private void enableLed(boolean value) {
    switch (device) {
      case HITECHNIC_NXT:
        colorSensor.enableLed(value);
        break;
      case ADAFRUIT:
        led.enable(value);
        break;
      case MODERN_ROBOTICS_I2C:
        colorSensor.enableLed(value);
        break;
    }
  }
}