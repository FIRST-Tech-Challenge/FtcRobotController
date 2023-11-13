/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * Demonstrates an empty iterative OpMode
 */
@Autonomous(name = "RevBot Autonomous", group = "Auto")
public class RevBotAuto extends OpMode {

  RevStarterRobotHardware robot       = new RevStarterRobotHardware(this);

  private ElapsedTime runtime = new ElapsedTime();
  private FirstVisionProcessor visionProcessor;
  private VisionPortal visionPortal;

  private FirstVisionProcessor.Selected selectedSide = FirstVisionProcessor.Selected.NONE;
  private boolean visionProcessingOn = false;
  /**
   * This method will be called once, when the INIT button is pressed.
   */
  @Override
  public void init() {
    robot.init();
    telemetry.addData("Status", "Initialized");
  }

  /**
   * This method will be called repeatedly during the period between when
   * the init button is pressed and when the play button is pressed (or the
   * OpMode is stopped).
   */
  @Override
  public void init_loop() {
    // TODO: Determine blue vs. red alliance and near or far position from the scoring board
  }

  /**
   * This method will be called once, when the play button is pressed.
   */
  @Override
  public void start() {

    runtime.reset();
    visionProcessor = new FirstVisionProcessor();
    visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
    visionProcessingOn = true;

  }

  /**
   * This method will be called repeatedly during the period between when
   * the play button is pressed and when the OpMode is stopped.
   */
  @Override
  public void loop() {
    if(visionProcessingOn){
      selectedSide = visionProcessor.getSelection();
      if (selectedSide != FirstVisionProcessor.Selected.NONE){
        visionPortal.stopStreaming();
        visionProcessingOn = false;
      }
    }

    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }

  /**
   * This method will be called once, when this OpMode is stopped.
   * <p>
   * Your ability to control hardware from this method will be limited.
   */
  @Override
  public void stop() {

  }
}
