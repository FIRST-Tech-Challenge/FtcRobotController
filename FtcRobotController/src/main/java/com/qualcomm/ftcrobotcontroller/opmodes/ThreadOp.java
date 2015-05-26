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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * ThreadOp
 * <p>
 * This OP mode shows an example of an op mode using threads.
 */
public class ThreadOp extends OpMode {

  /*
   * This class continually calculates the position of a servo
   */
  private class WaveRunnable implements Runnable {

    private static final double MIN_POSITION = 0.25;
    private static final double MAX_POSITION = 0.75;

    private static final double POSITION_DELTA = 0.005;

    // These variables are volatile to let Java know multiple threads will access it
    private volatile double position = 0.0;
    private volatile boolean running = true;

    private double direction = 1.0;

    @Override
    public void run() {

      DbgLog.msg("Starting up wave thread");

      try {
        running = true;
        while (running) {
          position += (POSITION_DELTA * direction);

          if (position > MAX_POSITION) {
            position = MAX_POSITION;
            direction *= -1;
          } else if (position < MIN_POSITION) {
            position = MIN_POSITION;
            direction *= -1;
          }

          // the servo won't get the update until it is ready
          servoA.setPosition(position);

          Thread.sleep(10);
        }
      } catch (InterruptedException e) {
        // allow thread to shut down when we receive an interrupted exception
      }

      running = false;

      DbgLog.msg("Stopping up wave thread");
    }

    public void shutdown() {
      running = false;
    }
  }

  WaveRunnable waveRunnable;
  Thread waveThread;
  Servo servoA;

  /*
   * Code to run when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */
  @Override
  public void start() {
    servoA = hardwareMap.servo.get("a");

    waveRunnable = new WaveRunnable();
    waveThread = new Thread(waveRunnable);
    waveThread.start();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    telemetry.addData("servo a", servoA.getPosition());
  }

  /*
   * Code to run when the op mode is first disabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
   */
  @Override
  public void stop() {
    if (waveRunnable != null) waveRunnable.shutdown();
  }
}
