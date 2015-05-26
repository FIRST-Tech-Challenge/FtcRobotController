
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

//import com.qualcomm.ftccommon.DbgLog;
//import com.qualcomm.hitechnic.HiTechnicUsbLegacyModule;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Test app.
 * <p>
 * For test purposes only.
 */
public class SoftwareRunToPosition extends OpMode {

  DcMotorController USB_DCMC;  // legacy NXT DC motor controller.
  DcMotor motorRight;
  DcMotor motorLeft;

  int leftPos = 0, rightPos=0;
  int targetLeftPos = 1440, targetRightPos = -1440;

  DcMotorController.DeviceMode devMode;

  /**
   * Constructor
   */
  public SoftwareRunToPosition() {

  }

  /*
   * Code to run when the op mode is first enabled goes here
   *
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */
  @Override
  public void start() {

    DbgLog.msg("Beginning SoftwareRunToPosition");
    // get a handle to the legacy dc motor controller.
    USB_DCMC = hardwareMap.dcMotorController.get("MC");

    // get the motors attached to channels 1 and 2 of the NXT DC motor controller.
    motorRight = hardwareMap.dcMotor.get("motor2");
    motorLeft = hardwareMap.dcMotor.get("motor1");
    motorRight.setDirection(DcMotor.Direction.REVERSE);

    // Nxt devices start up in "write" mode by default
    devMode = DcMotorController.DeviceMode.WRITE_ONLY;

    // set to use encoders.
    motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);


  }

  boolean setup = true;
  /*
   * This method will be called repeatedly in a loop
   *
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
   */
  @Override
  public void loop() {

    DbgLog.error("Beginning of loop... targetLeftPos: " + targetLeftPos + ", targetRightPos: " + targetRightPos);
    if (setup) {
      DbgLog.error("setup...");
      // set to use encoders.
      motorLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
      motorRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
      setup = false;
    }

    leftPos = motorLeft.getCurrentPosition();
    rightPos = motorRight.getCurrentPosition();
    DbgLog.error("leftPos: " + leftPos + ", rightPos: " + rightPos);

    if (leftPos < targetLeftPos) {
      motorLeft.setPower(0.03f);
    }
    if (rightPos > targetRightPos) {
      motorRight.setPower(0.03f);
    }

    if (rightPos <= targetRightPos) {
      motorRight.setPower(0.0f);
      DbgLog.msg("Right reached the goal!");
    }
    if (leftPos >= targetLeftPos) {
      DbgLog.msg("Left reached the goal!");
      motorLeft.setPower(0.0f);
    }
    telemetry.addData("Text", "*** Robot Data***");

  }

  /*
   * Code to run when the op mode is first disabled goes here
   *
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
   */
  @Override
  public void stop() {
    // set back to write only before we leave.
    USB_DCMC.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
    devMode = DcMotorController.DeviceMode.WRITE_ONLY;
  }

  /*
   * Observations:
   * 1440 causes the wheel to turn 4 times.
   */

}
