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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * Demonstrates an empty iterative OpMode
 */
@Autonomous(name = "RevBot Autonomous", group = "Auto")
public class RevBotAuto extends OpMode {

  RevStarterRobotHardware robot       = new RevStarterRobotHardware(this);

  private ElapsedTime runtime = new ElapsedTime();
  private FirstVisionProcessor visionProcessor;
  private AprilTagProcessor aprilTag;

  // Used to hold the data for a detected AprilTag
  private AprilTagDetection desiredTag = null;
  private VisionPortal visionPortal;

  private ScoringElementLocation selectedSide = ScoringElementLocation.UNKNOWN;

  private State state = State.START;

  private double lastTime;

  private Alliance alliance = Alliance.UNKNOWN;

  private FieldPosition fieldPosition = FieldPosition.UNKNOWN;
  private int desiredTagId;

  final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
  //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
  //  applied to the drive motors to correct the error.
  //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
  final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
  final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
  final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
  final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)


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
  }
  /**
   * This method will be called once, when the INIT button is pressed.
   */
  @Override
  public void init() {
    robot.init();
    state = State.DETERMINE_ALLIANCE;
    initVisionPortal();
    telemetry.addData("Status", "Initialized");
  }

  /**
   * This method will be called repeatedly during the period between when
   * the init button is pressed and when the play button is pressed (or the
   * OpMode is stopped).
   */
  @Override
  public void init_loop() {
    // Determine blue vs. red alliance
    alliance = getAllianceFromKeyEntry();
    // Determine near or far position from the scoring board
    fieldPosition = getFieldPositionFromKeyEntry();

    if (alliance != Alliance.UNKNOWN && fieldPosition != FieldPosition.UNKNOWN) {
      // We know both alliance and field position
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
    if (state == State.IDENTIFY_SPIKE){
      visionProcessor.SetAlliance(alliance);
    }
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
          telemetry.addData("selectedSide: " , selectedSide.toString());
          state = State.PLACE_PIXEL_ON_SPIKE;
          lastTime = getRuntime();
        }
        break;
      case PLACE_PIXEL_ON_SPIKE:
        state = State.MOVE_TO_SCOREBOARD;
        lastTime = getRuntime();

        break;
      case MOVE_TO_SCOREBOARD:
        desiredTagId = getDesiredTagId(alliance, selectedSide);
        visionPortal.setProcessorEnabled(aprilTag, true);
        // TODO: Step 1: Move based on approximation - time/ encoder / gyro.
        // Keep looking for the desired aprilTag
        boolean targetFound = findDesiredAprilTag(desiredTagId);

        // Step 2: Move based on AprilTag location once it is found.
        if (!targetFound){
          break;
        }
        boolean targetReached = driveToAprilTagTarget(desiredTag);
        if (!targetReached){
          break;
        }
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

  /**
   * Init vision portal. AprilTag and VisionProcessor
   */
  private void initVisionPortal() {
    visionProcessor = new FirstVisionProcessor();
    // Create the AprilTag processor by using a builder.
    aprilTag = new AprilTagProcessor.Builder().build();
    aprilTag.setDecimation(1);
    visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessors(visionProcessor, aprilTag)
            .build();
    // At the beginning, vision processor on and aprilTag processor off
    visionPortal.setProcessorEnabled(aprilTag, false);
    visionPortal.setProcessorEnabled(visionProcessor, true);
  }

  // TODO: Implement this
  private int getDesiredTagId(Alliance alliance, ScoringElementLocation selectedSide){
    if (alliance == Alliance.UNKNOWN || selectedSide == ScoringElementLocation.UNKNOWN){
      return -1;
    }

    return 5;
  }

  // TODO: Implement this
  private Alliance getAllianceFromKeyEntry(){
    return Alliance.UNKNOWN;
  }


  // TODO: Implement this
  private FieldPosition getFieldPositionFromKeyEntry(){
    return FieldPosition.UNKNOWN;
  }

  boolean findDesiredAprilTag(int desiredTagId){
    boolean targetFound = false;
    desiredTag  = null;

    // Step through the list of detected tags and look for a matching tag
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    for (AprilTagDetection detection : currentDetections) {
      // Look to see if we have size info on this tag.
      if (detection.metadata != null) {
        //  Check to see if we want to track towards this tag.
        if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
          // Yes, we want to use this tag.
          targetFound = true;
          desiredTag = detection;
          break;  // don't look any further.
        } else {
          // This tag is in the library, but we do not want to track it right now.
          telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
        }
      } else {
        // This tag is NOT in the library, so we don't have enough information to track to it.
        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
      }
    }
    if (targetFound) {
      telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
      telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
      telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
    }
    return targetFound;
  }

  boolean driveToAprilTagTarget(AprilTagDetection desiredTag){
    if (desiredTag == null){
      return false;
    }

    // Determine heading and range error so we can use them to control the robot automatically.
    double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
    double  headingError = desiredTag.ftcPose.bearing;

    // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
    double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
    double turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

    telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
    telemetry.addData("Auto","Range Err %5.2f, Heading Err %5.2f", rangeError, headingError);
    robot.driveRobot(drive, turn);

    // TODO: Tune this
    if (rangeError < 2) {
      return true;
    }

    return false;
  }

}
