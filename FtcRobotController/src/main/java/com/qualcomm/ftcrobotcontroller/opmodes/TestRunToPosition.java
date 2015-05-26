/* Copyright (c) 2014 Qualcomm Technologies Inc

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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TEST OP Mode
 * <p>
 * USED FOR TESTING ONLY
 */
public class TestRunToPosition extends OpMode {

  ElapsedTime timer = new ElapsedTime();

  DcMotorController wheelController;
  int GOAL = 1440;

  DcMotor left;
  DcMotor right;

  boolean runToPosition = true;
  boolean readEncoders = false;

  // how long to hold before the next action
  final static double HOLD_POSITION = 0.5; // in seconds
  private double pauseTime = 0.0;

  DcMotorController.DeviceMode devMode = DcMotorController.DeviceMode.READ_WRITE;

  /*
   * Code to run when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */
  @Override
  public void start() {

    DbgLog.msg("****** TEST RUN TO POSITION: start");

    left = hardwareMap.dcMotor.get("motor1");
    right = hardwareMap.dcMotor.get("motor2");
    right.setDirection(DcMotor.Direction.REVERSE);

    // NXT devices start up in DeviceMode.WRITE_ONLY;
    wheelController = hardwareMap.dcMotorController.get("MC");

    // Reset encoders to zero.
    left.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    right.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

    // calculate how long we should hold the current position
    pauseTime = time + HOLD_POSITION;
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {

    if (time > pauseTime) {

      if (runToPosition) {
        devMode = wheelController.getMotorControllerDeviceMode();
        DbgLog.error("Running to position.... current mode: " + devMode);
        right.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        left.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        right.setPower(0.05f);
        left.setPower(0.05f);

        right.setTargetPosition(GOAL);
        left.setTargetPosition(GOAL);

        runToPosition = false;
        readEncoders = true;
      }
      else if (readEncoders) {
        DbgLog.error("Switching to read mode... current mode: " + devMode);
        wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        readEncoders = false;
      }

      // pause for HOLD_POSITION seconds
      pauseTime = time + HOLD_POSITION;
      devMode = wheelController.getMotorControllerDeviceMode();
      DbgLog.error("devMode: " + devMode);
      if (allowedToRead()) {
        telemetry.addData("left.currentEncoder: ", left.getCurrentPosition());
        telemetry.addData("left.target: ", left.getTargetPosition());
        telemetry.addData("right.currentEncoder: ", right.getCurrentPosition());
        telemetry.addData("right.target: ", right.getTargetPosition());
      }
    }

    timer.reset();
  }

  private boolean allowedToRead() {
    return devMode.equals(DcMotorController.DeviceMode.READ_ONLY);
  }

  /*
   * Code to run when the op mode is first disabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
   */
  @Override
  public void stop() {
    DbgLog.msg("****** TEST RUN TO POSITION OP: stop");
  }

}
