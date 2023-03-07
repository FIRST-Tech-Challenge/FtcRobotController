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

import android.os.SystemClock;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc.navXPerformanceMonitor;

import java.text.DecimalFormat;

/**
 * navX-Micro Performance Tuning Op
 *
 * This opmode provides insight into the peformance of the communication
 * with the navX-Model sensor over the I2C bus via the Core Device Interface
 * Module.  Since the Android operating system is not a real-time
 * operating system, and since communication w/the navX-Model sensor is
 * occurring over a wifi-direct network which can be prone to interference,
 * the actual performance (update rate) achieved may be less than
 * one might expect.
 *
 * Since the navX-Model devices integrate sensor data onboard, to achieve
 * the best performance for device control methods like a PID controller
 * that work best with constant-time updates, the strategy is to:
 *
 * 1) Configure the navX-Model device to a high update rate (e.g., 50Hz)
 * 2) Using this performance-tuning Op-Mode (with all other
 * sensors connected, just as your robot will be configured for normal
 * use) observe the "effective" update rate, and the number of missed
 * samples over the last minute.
 * 3) Lower the navX-Model device update rate until the number of missed
 * samples over the last minute reaches zero.
 */
@TeleOp(name = "Concept: navX Performance Tuning", group = "Concept")
@Disabled
public class ConceptNavXPerformanceTuningOp extends OpMode {

  private AHRS navx_device;
  private navXPerformanceMonitor navx_perfmon;
  private byte sensor_update_rate_hz = 40;
  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void init() {
    AHRS.setLogging(true);
    navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
            AHRS.DeviceDataType.kProcessedData,
            sensor_update_rate_hz);
    navx_perfmon = new navXPerformanceMonitor(navx_device);
  }

@Override
  public void start() {
    navx_device.registerCallback(navx_perfmon);
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
      telemetry.addData("1 navX-Device...............:", connected ?
              "Connected" : "Disconnected" );
      String gyrocal, motion;
      DecimalFormat df = new DecimalFormat("#.##");

      telemetry.addData("2 Sensor Rate (Hz)...", Byte.toString(navx_device.getActualUpdateRate()));
      telemetry.addData("3 Transfer Rate (Hz).", Integer.toString(navx_device.getCurrentTransferRate()));
      telemetry.addData("4 Delivvered Rate (Hz)", Integer.toString(navx_perfmon.getDeliveredRateHz()));
      telemetry.addData("5 Missed Samples.....", Integer.toString(navx_perfmon.getNumMissedSensorTimestampedSamples()));
      telemetry.addData("6 Duplicate Samples..", Integer.toString(navx_device.getDuplicateDataCount()));
      telemetry.addData("7 Sensor deltaT (ms).", Long.toString(navx_perfmon.getLastSensorTimestampDeltaMS()));
      telemetry.addData("8 System deltaT (ms).", Long.toString(navx_perfmon.getLastSystemTimestampDeltaMS()));
  }
}
