/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

package org.firstinspires.ftc.teamcode.android.OpModes;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc.AHRS;

import java.text.DecimalFormat;

/**
 *  navX-Micro Processed Data Mode Op
 * <p>
 * Acquires motion-in-progress indicator from navX-Micro
 * and displays it in the Robot DriveStation
 * as telemetry data.
 */
@TeleOp(name = "Concept: navX Motion Detection", group = "Concept")
@Disabled
public class ConceptNavXMotionDetectionOp extends OpMode {

  private String startDate;
  private ElapsedTime runtime = new ElapsedTime();
  private AHRS navx_device;

  @Override
  public void init() {
    navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
            AHRS.DeviceDataType.kProcessedData);
  }

  @Override
  public void stop() {
    navx_device.close();
  }
  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    telemetry.addData("navX Op Init Loop", runtime.toString());
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {

      boolean connected = navx_device.isConnected();
      telemetry.addData("1 navX-Device", connected ?
              "Connected" : "Disconnected" );
      String gyrocal, motion;
      DecimalFormat df = new DecimalFormat("#.##");

      if ( connected ) {
          gyrocal = (navx_device.isCalibrating() ?
                  "CALIBRATING" : "Calibration Complete");
          motion = (navx_device.isMoving() ? "Moving" : "Not Moving");
          if ( navx_device.isRotating() ) {
              motion += ", Rotating";
          }
      } else {
          gyrocal =
            motion = "-------";
      }
      telemetry.addData("2 GyroAccel", gyrocal );
      telemetry.addData("3 Motion", motion);
  }

}
