/*
 * Copyright (c) 2015 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CompassCalibration extends OpMode {

  // how long to hold before the next action
  final static double HOLD_POSITION = 3.0; // in seconds

  // wheel speed
  final static double MOTOR_POWER = 0.2; // scale from 0 to 1

  // Turn around at least twice in 20 seconds.
  private double turnTime = 20.0;

  private boolean keepTurning = true;
  private boolean returnToMeasurementMode = false;
  private boolean monitorCalibrationSuccess = false;

  // when paused time as passed, we will switch back to measurement mode.
  double pauseTime = 2.0;

  CompassSensor compass;
  DcMotor motorRight;
  DcMotor motorLeft;

  @Override
  public void init() {
    compass = hardwareMap.compassSensor.get("compass");
    motorRight = hardwareMap.dcMotor.get("right");
    motorLeft = hardwareMap.dcMotor.get("left");

    motorRight.setDirection(DcMotor.Direction.REVERSE);

    // Set the compass to calibration mode
    compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
    telemetry.addData("Compass", "Compass in calibration mode");

    // calculate how long we should hold the current position
    pauseTime = time + HOLD_POSITION;
  }

  @Override
  public void loop() {

    // make sure pauseTime has passed before we take any action
    if (time > pauseTime) {

      // have we turned around at least twice in 20 seconds?
      if (keepTurning) {

        telemetry.addData("Compass", "Calibration mode. Turning the robot...");
        DbgLog.msg("Calibration mode. Turning the robot...");

        // rotate the robot towards our goal direction
        motorRight.setPower(-MOTOR_POWER);
        motorLeft.setPower(MOTOR_POWER);

        // Only turn for 20 seconds (plus the two second pause at the beginning)
        if (time > turnTime + HOLD_POSITION) {
          keepTurning = false;
          returnToMeasurementMode = true;
        }
      } else if (returnToMeasurementMode) {

        telemetry.addData("Compass", "Returning to measurement mode");
        DbgLog.msg("Returning to measurement mode");
        motorRight.setPower(0.0);
        motorLeft.setPower(0.0);

        // change compass mode
        compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);

        // set a new pauseTime
        pauseTime = time + HOLD_POSITION;

        returnToMeasurementMode = false;
        monitorCalibrationSuccess = true;
        telemetry.addData("Compass", "Waiting for feedback from sensor...");

      } else if (monitorCalibrationSuccess) {

        String msg = calibrationMessageToString(compass.calibrationFailed());
        telemetry.addData("Compass",  msg);

        if (compass.calibrationFailed()) {
          DbgLog.error("Calibration failed and needs to be re-run");
        } else {
          DbgLog.msg(msg);
        }

      }
      // set a new pauseTime
      pauseTime = time + HOLD_POSITION;
    }
  }

  private String calibrationMessageToString(boolean failed) {
    if (failed){ return "Calibration Failed!"; }
    else { return "Calibration Succeeded." ; }
  }
}
