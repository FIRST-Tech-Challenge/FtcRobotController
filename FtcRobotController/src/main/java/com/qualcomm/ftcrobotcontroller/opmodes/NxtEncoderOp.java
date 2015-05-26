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

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * NxtEncoderOp
 * <p>
 * Very simple op mode that demonstrates a state machine with the NXT motor controllers.
 * Should move forward, and then move backwards to its original location.
 */
public class NxtEncoderOp extends OpMode {

  DcMotorController.DeviceMode devMode;
  DcMotorController wheelController;
  DcMotor motorRight;
  DcMotor motorLeft;

  int motorRightCurrentEncoder;
  int motorLeftCurrentEncoder;
  int motorRightTargetEncoder;
  int motorLeftTargetEncoder;

  DcMotorController.RunMode motorRightRunMode;
  DcMotorController.RunMode motorLeftRunMode;

  int numOpLoops = 1;

  State state;
  public enum State {
    STATE_ZERO,
    STATE_ONE,
    STATE_TWO,
    STATE_THREE
  }

  int firstTarget = 1440*6;
  int secondTarget = 0;

  /**
   * Constructor
   */
  public NxtEncoderOp() {

  }

  /*
   * Code to run when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */
  @Override
  public void start() {

    state = State.STATE_ZERO;

    motorRight = hardwareMap.dcMotor.get("motor_2");
    motorLeft = hardwareMap.dcMotor.get("motor_1");

    wheelController = hardwareMap.dcMotorController.get("wheels");
    devMode = DcMotorController.DeviceMode.WRITE_ONLY;

    motorLeft.setDirection(DcMotor.Direction.REVERSE);

    // set the mode
    // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
    motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

    motorLeftRunMode = DcMotorController.RunMode.RUN_USING_ENCODERS;
    motorRightRunMode = DcMotorController.RunMode.RUN_USING_ENCODERS;
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    DbgLog.error("mode: " + devMode);

    // The op mode should only use "write" methods (setPower, setChannelMode, etc) while in
    // WRITE_ONLY mode.
    if (allowedToWrite()) {
      switch (state) {

        case STATE_ZERO:
          if (motorRightRunMode != DcMotorController.RunMode.RESET_ENCODERS ||
              motorLeftRunMode != DcMotorController.RunMode.RESET_ENCODERS) {
            motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
          } else {
            state = State.STATE_ONE;
          }
          break;

        case STATE_ONE:
          if (bothEncodersZero() &&
              motorsInCorrectMode(DcMotorController.RunMode.RUN_TO_POSITION)) {
            state = State.STATE_TWO;
          } else {
            motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
          }
          break;

        case STATE_TWO:
          motorLeft.setTargetPosition(firstTarget);
          motorRight.setTargetPosition(-firstTarget);

          motorLeft.setPower(1.0);
          motorRight.setPower(1.0);

          if (motorLeftTargetEncoder == firstTarget && motorRightTargetEncoder == -firstTarget) {
            state = State.STATE_THREE;
          }
          break;

        case STATE_THREE:
          if (withinMarginOfError(firstTarget, motorLeftCurrentEncoder) &&
              withinMarginOfError(-firstTarget, motorRightCurrentEncoder)) {

            motorLeft.setTargetPosition(secondTarget);
            motorRight.setTargetPosition(secondTarget);

          }

          motorLeft.setPower(1.0);
          motorRight.setPower(1.0);
          break;
      }
    }

    // To read any values from the NXT controllers, we need to switch into READ_ONLY mode.
    // It takes time for the hardware to switch, so you can't switch modes within one loop of the
    // op mode. Every 17th loop, this op mode switches to READ_ONLY mode.
    if (numOpLoops % 17 == 0) {
      // Note: If you are using the NxtDcMotorController, you need to switch into "read" mode
      // before doing a read, and into "write" mode before doing a write. This is because
      // the NxtDcMotorController is on the I2C interface, and can only do one at a time. If you are
      // using the USBDcMotorController, there is no need to switch, because USB can handle reads
      // and writes without changing modes. The NxtDcMotorControllers start up in "write" mode.
      // This method does nothing on USB devices, but is needed on Nxt devices.
      wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
    }

    // If we've switched to read mode, we can read data from the NXT device.
    // Only necessary on NXT devices.
    if (devMode == DcMotorController.DeviceMode.READ_ONLY) {

      // Update the reads after some loops, when the command has successfully propagated through.
      telemetry.addData("motorLeft Power", motorLeft.getPower());
      telemetry.addData("motorRight power", motorRight.getPower());

      motorRightCurrentEncoder = motorRight.getCurrentPosition();
      motorLeftCurrentEncoder = motorLeft.getCurrentPosition();

      telemetry.addData("right curr enc", motorRightCurrentEncoder);
      telemetry.addData("left curr enc", motorLeftCurrentEncoder);

      motorRightRunMode = motorRight.getChannelMode();
      motorLeftRunMode = motorLeft.getChannelMode();

      telemetry.addData("right runmode", "right runmode: " + motorRightRunMode.toString());
      telemetry.addData("left runmode", "left runmode: " + motorLeftRunMode.toString());

      motorLeftTargetEncoder = motorLeft.getTargetPosition();
      motorRightTargetEncoder = motorRight.getTargetPosition();

      telemetry.addData("right target", motorRightTargetEncoder);
      telemetry.addData("left target", motorLeftTargetEncoder);

      // Only needed on Nxt devices, but not on USB devices
      wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

      // Reset the loop
      numOpLoops = 0;
    }

    // Update the current devMode
    devMode = wheelController.getMotorControllerDeviceMode();
    numOpLoops++;

  }

  /*
   * Code to run when the op mode is first disabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
   */
  @Override
  public void stop() {
    if (allowedToWrite()) {
      motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
      motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

  }

  private boolean allowedToWrite(){
    return (devMode == DcMotorController.DeviceMode.WRITE_ONLY);
  }

  private boolean withinMarginOfError(int goal, int value) {
    int lowerMargin = goal - 2;
    int upperMargin = goal + 2;
    return (value >= lowerMargin && value <= upperMargin);
  }

  private boolean bothEncodersZero() {
    return motorRightCurrentEncoder == 0 && motorLeftCurrentEncoder == 0;
  }

  private boolean motorsInCorrectMode(DcMotorController.RunMode runMode) {
    return motorLeftRunMode == runMode && motorRightRunMode == runMode;
  }

}
