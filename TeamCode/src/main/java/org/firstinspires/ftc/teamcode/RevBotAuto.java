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
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * Demonstrates an empty iterative OpMode
 */
@Autonomous(name = "RevBot Autonomous", group = "Auto")
public class RevBotAuto extends OpMode {

  RevStarterRobotHardware robot       = new RevStarterRobotHardware(this);

  private ElapsedTime runtime = new ElapsedTime();
  private FirstVisionProcessor visionProcessor;
  private AprilTagProcessor aprilTag;
  private VisionPortal visionPortal;

  private ScoringElementLocation selectedSide = ScoringElementLocation.UNKNOWN;

  private boolean isBlue = false;
  private boolean isFar = false;

  private State state = State.START;

  private double lastTime;

  private Alliance alliance;

  /// State machine
  enum State {
    START,
    DETERMINE_ALLIANCE,
    IDENTIFY_SPIKE,
    PLACE_PIXEL_ON_SPIKE,
    MOVE_TO_SCOREBOARD,
    SCORE_PIXEL,
    PARK,
    END
  };
  /**
   * This method will be called once, when the INIT button is pressed.
   */
  @Override
  public void init() {
    robot.init();
    state = State.DETERMINE_ALLIANCE;
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

    if (alliance != Alliance.UNKNOWN) {
      state = State.IDENTIFY_SPIKE;
    }
  }

  /**
   * This method will be called once, when the play button is pressed.
   */
  @Override
  public void start() {

    runtime.reset();
    lastTime = getRuntime();
    visionProcessor = new FirstVisionProcessor();
    visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessors(visionProcessor, aprilTag)
            .build();
    // At the beginning, vision processor on and aprilTag processor off
    visionPortal.setProcessorEnabled(aprilTag, false);
    visionPortal.setProcessorEnabled(visionProcessor, true);
    visionProcessor.SetAlliance(alliance);
  }

  /**
   * This method will be called repeatedly during the period between when
   * the play button is pressed and when the OpMode is stopped.
   */
  @Override
  public void loop() {

    switch (state) {
      case START:
      case DETERMINE_ALLIANCE:
        telemetry.addData("Unexpected State: " , state.toString());
        break;
      case IDENTIFY_SPIKE:
        selectedSide = visionProcessor.getSelection();
        if (selectedSide != ScoringElementLocation.UNKNOWN){
          visionPortal.stopStreaming();
          visionPortal.setProcessorEnabled(visionProcessor, false);
          state = State.PLACE_PIXEL_ON_SPIKE;
          lastTime = getRuntime();
        }
        break;
      case PLACE_PIXEL_ON_SPIKE:
        state = State.MOVE_TO_SCOREBOARD;
        lastTime = getRuntime();

        break;
      case MOVE_TO_SCOREBOARD:
        visionPortal.setProcessorEnabled(aprilTag, true);
        state = State.SCORE_PIXEL;
        lastTime = getRuntime();

        visionPortal.setProcessorEnabled(aprilTag, false);
        break;
      case SCORE_PIXEL:
        state = State.PARK;
        lastTime = getRuntime();

        break;
      case PARK:
        state = State.END;
        lastTime = getRuntime();
        break;
      case END:
        break;
      default:
        telemetry.addData("Invalid State: " , state.toString());
        break;
    }

    telemetry.addData("State: " , state.toString());
    telemetry.addData("Runtime", getRuntime());
    telemetry.addData("Time in State", getRuntime() - lastTime);
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
