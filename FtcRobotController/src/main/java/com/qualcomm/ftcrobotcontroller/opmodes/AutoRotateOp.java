/*
 * Copyright (c) 2014 Qualcomm Technologies Inc
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

public class AutoRotateOp extends OpMode {

  // we will consider our goal reached if we are +/- ERROR_MARGIN degrees
  final static double ERROR_MARGIN = 5.0; // in degrees

  // how long to hold our goal position
  final static double HOLD_POSITION = 3.0; // in seconds

  // wheel speed while moving to our goal
  final static double MOTOR_POWER = 0.2; // scale from 0 to 1

  // each goal direction in degrees
  final static double GOALS[] = { 0.0, 90.0, 180.0, 270.0 };

  // index of the current goal
  int currentGoal = 0;

  // when paused time as passed, we will proceed to the next goal
  double pauseTime = 0.0;

  CompassSensor compass;
  DcMotor motorRight;
  DcMotor motorLeft;

  public AutoRotateOp() {

  }

  @Override
  public void start() {
    compass = hardwareMap.compassSensor.get("compass");
    motorRight = hardwareMap.dcMotor.get("right");
    motorLeft = hardwareMap.dcMotor.get("left");

    motorRight.setDirection(DcMotor.Direction.REVERSE);

    // calculate how long should we hold the current position
    pauseTime = time + HOLD_POSITION;
  }

  @Override
  public void loop() {

    // make sure pauseTime has passed before we take any action
    if (time > pauseTime) {

      // get our current direction
      double facing = compass.getDirection();

      // have we reached our goal direction?
      if (hasReachedDegree(GOALS[currentGoal], facing) == false) {
        // no, move toward our goal
        DbgLog.msg("moving from " + facing + " toward " + GOALS[currentGoal]);

        // rotate the robot towards our goal direction
        motorRight.setPower(-MOTOR_POWER);
        motorLeft.setPower(MOTOR_POWER);
      } else {
        // yes, set a new goal
        DbgLog.msg("holding " + facing);

        motorRight.setPower(0.0);
        motorLeft.setPower(0.0);

        // set a new goal direction
        currentGoal = (currentGoal + 1) % GOALS.length;

        // set a new pauseTime
        pauseTime = time + HOLD_POSITION;
      }
    }
  }

  // check if we reached our target degree, +/- ERROR_MARGIN
  private boolean hasReachedDegree(double target, double current) {
    return (
        (Math.abs(target - (current      ))) < ERROR_MARGIN ||
        (Math.abs(target - (current - 360))) < ERROR_MARGIN ||
        (Math.abs(target - (current + 360))) < ERROR_MARGIN);
  }

  @Override
  public void stop() {

  }

}
