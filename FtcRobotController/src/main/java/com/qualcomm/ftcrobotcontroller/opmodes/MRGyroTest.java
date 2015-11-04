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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * the Modern Robotics Gyro.
 *
 * The op mode assumes that the gyro sensor
 * is configured with a name of "gyro".
 *
 *
 *
 */
public class MRGyroTest extends LinearOpMode {


  @Override
  public void runOpMode() throws InterruptedException {

    GyroSensor sensorGyro;
    int xVal, yVal, zVal = 0;
    int heading = 0;

    // write some device information (connection info, name and type)
    // to the log file.
    hardwareMap.logDevices();

    // get a reference to our GyroSensor object.
    sensorGyro = hardwareMap.gyroSensor.get("gyro");

    // calibrate the gyro.
    sensorGyro.calibrate();

    // wait for the start button to be pressed.
    waitForStart();

    // make sure the gyro is calibrated.
    while (sensorGyro.isCalibrating())  {
      Thread.sleep(50);
    }

    while (opModeIsActive())  {
      // if the A and B buttons are pressed, reset Z heading.
      if(gamepad1.a && gamepad1.b)  {
        // reset heading.
        sensorGyro.resetZAxisIntegrator();
      }

      // get the x, y, and z values (rate of change of angle).
      xVal = sensorGyro.rawX();
      yVal = sensorGyro.rawY();
      zVal = sensorGyro.rawZ();

      // get the heading info.
      // the Modern Robotics' gyro sensor keeps
      // track of the current heading for the Z axis only.
      heading = sensorGyro.getHeading();

      telemetry.addData("1. x", String.format("%03d", xVal));
      telemetry.addData("2. y", String.format("%03d", yVal));
      telemetry.addData("3. z", String.format("%03d", zVal));
      telemetry.addData("4. h", String.format("%03d", heading));

      Thread.sleep(100);
    }
  }
}
