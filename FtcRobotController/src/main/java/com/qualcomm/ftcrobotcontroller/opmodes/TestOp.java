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
public class TestOp extends OpMode {

  ElapsedTime timer = new ElapsedTime();

  DcMotorController wheelController;
  DcMotorController flagArmController;

  DcMotor left;
  DcMotor right;

  DcMotor flag;
  DcMotor arm;

  Servo servoA;
  Servo servoB;
  //Servo servoC;

  //IrSeekerSensor irSeekerSensor;
  //LightSensor lightSensor;

  double servoPosition = 0.0;

  // how long to hold before the next action
  final static double HOLD_POSITION = 0.5; // in seconds
  private double pauseTime = 0.0;

  DcMotorController.DeviceMode devMode = DcMotorController.DeviceMode.READ_WRITE;

  private boolean attemptReadInWriteMode = true;
  private boolean switchToReadMode = false;
  private boolean attemptWriteInReadMode = false;
  private boolean attemptToSetInvalidMode = false;
  private boolean resetEncoders = false;
  private boolean runToPosition = false;
  private boolean readEncoders = false;

  /*
   * Code to run when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */
  @Override
  public void start() {

    DbgLog.msg("****** TEST OP: start");

    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
    right.setDirection(DcMotor.Direction.REVERSE);

    wheelController = hardwareMap.dcMotorController.get("wheels");
    flagArmController = hardwareMap.dcMotorController.get("flagArm");

    flag = hardwareMap.dcMotor.get("flag");
    arm = hardwareMap.dcMotor.get("arm");

    servoA = hardwareMap.servo.get("a");
    servoB = hardwareMap.servo.get("b");
    //servoC = hardwareMap.servo.get("c");

    //irSeekerSensor = hardwareMap.irSeekerSensor.get("ir_seeker");
    //lightSensor = hardwareMap.lightSensor.get("light");

    //lightSensor.enableLed(true);

    // calculate how long we should hold the current position
    pauseTime = time + HOLD_POSITION;
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {

    double curPosLeft = 0.0;
    DcMotorController.RunMode mode = DcMotorController.RunMode.RESET_ENCODERS;


    servoA.setPosition(gamepad1.left_trigger);
    servoB.setPosition(gamepad1.right_trigger);

    //left.setPower(gamepad1.left_stick_y);
    //right.setPower(gamepad1.right_stick_y);

    if (time > pauseTime) {
      if(attemptReadInWriteMode) {
        telemetry.addData("attemptReadInWriteMode", "attemptReadInWriteMode");
        DbgLog.error("attemptReadInWriteMode #####################################");
        try {
          curPosLeft = left.getCurrentPosition();
          devMode = wheelController.getMotorControllerDeviceMode();

        } catch (IllegalArgumentException e) {
          DbgLog.error("Successfully caught the error of reading while in write mode." + e.toString());
          attemptReadInWriteMode = false;
          switchToReadMode = true;

        }
      }
      else if (switchToReadMode) {
        telemetry.addData("switchToReadMode", "switchToReadMode");
        DbgLog.error("switchToReadMode ##########################");
        wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY); // sets the mode for the whole controller
        curPosLeft = left.getCurrentPosition();
        devMode = wheelController.getMotorControllerDeviceMode();

        DbgLog.error("curPosLeft: " + curPosLeft);
        DbgLog.error("devMode: " + devMode);

        switchToReadMode = false;
        attemptWriteInReadMode = true;

      }
      else if (attemptWriteInReadMode) {
        telemetry.addData("attemptWriteInReadMode", "attemptWriteInReadMode");
        DbgLog.error("attempt Write in Read Mode ##################################");

        devMode = wheelController.getMotorControllerDeviceMode();
        DbgLog.error("devmode: " + devMode);
        try {
          right.setTargetPosition(400);
        } catch (IllegalArgumentException e) {
          DbgLog.error("Successfully caught the error of writing in read mode: " + e.toString());
        }

        wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        attemptWriteInReadMode = false;
        attemptToSetInvalidMode = true;

      }
      else if (attemptToSetInvalidMode) {
        DbgLog.error("attemptToSetInvalidMode ############################");
        try {
          wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE);
        } catch (IllegalArgumentException e) {
          DbgLog.error("Successfully threw error on switch to invalid mode: " + e.toString());
          attemptToSetInvalidMode = false;
          resetEncoders = true;
        }
      }
      else if (resetEncoders) {
        DbgLog.error("Reseting encoders.... current mode: " + devMode);
        right.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        left.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);



        resetEncoders = false;
        runToPosition = true;
      }
      else if (runToPosition) {

        DbgLog.error("Running to position.... current mode: " + devMode);
        right.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        left.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        right.setPower(1.0);
        left.setPower(1.0);

        right.setTargetPosition(2);
        left.setTargetPosition(2);

        runToPosition = false;
        readEncoders = true;
      }
      else if (readEncoders) {
        DbgLog.error("Reading the encoders... current mode: " + devMode);
        wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        readEncoders = false;
      }

      pauseTime = time + HOLD_POSITION;
      devMode = wheelController.getMotorControllerDeviceMode();

      DbgLog.error("devMode: " + devMode);
    }

    if (gamepad1.x) {
      flag.setPower(0.10);
    } else {
      flag.setPower(0.0);
    }

    if (gamepad1.y) {
      arm.setPower(0.20);
    } else if (gamepad1.a) {
      arm.setPower(-0.05);
    } else {
      arm.setPower(0.0);
    }

    if (gamepad1.b) {
      servoPosition += 0.05;
    } else {
      servoPosition += -0.05;
    }
    if (servoPosition > 1.0) servoPosition = 1.0;
    if (servoPosition < 0.0) servoPosition = 0.0;
    //servoC.setPosition(servoPosition);

    telemetry.addData("devMode: ", devMode.toString());
    telemetry.addData("curPos: ", curPosLeft);
    //telemetry.addData("mode: ", mode.toString());


    telemetry.addData("servo a", servoA.getPosition());
    telemetry.addData("servo b", servoB.getPosition());
    telemetry.addData("runtime", time);
    //telemetry.addData("ir seeker strength", irSeekerSensor.getStrength());
    //telemetry.addData("ir seeker angle", irSeekerSensor.getAngle());
    //telemetry.addData("light", lightSensor.getLightLevel());
    telemetry.addData("timer", timer.time());

    timer.reset();
  }

  /*
   * Code to run when the op mode is first disabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
   */
  @Override
  public void stop() {
    DbgLog.msg("****** TEST OP: stop");
  }

}
