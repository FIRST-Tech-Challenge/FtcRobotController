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
 * RevBot Autonomous code
 */
@Autonomous(name = "RevBot Autonomous", group = "Auto")
public class RevBotAuto extends OpMode {

  RevStarterRobotHardware robot = new RevStarterRobotHardware(this);

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
    // Initialize
    START,
    // Determine alliance and near/far side using gamepad entry
    DETERMINE_ALLIANCE,
    // Identify if the custom game element is placed on left/center/right using computer vision
    IDENTIFY_SPIKE,
    // Drive the robot to drag the pixel with it to be placed on the correct spike mark (red/blue tape)
    PLACE_PIXEL_ON_SPIKE,
    // Move the robot towards scoreboard
    MOVE_TO_SCOREBOARD,
    //
    SCORE_PIXEL,
    //
    PARK,
    //
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
    if (alliance == Alliance.UNKNOWN){
      alliance = getAllianceFromKeyEntry();
    }
    telemetry.addData("Alliance", alliance.toString());
    // Determine near or far position from the scoring board
    fieldPosition = getFieldPositionFromKeyEntry();
    if (fieldPosition == FieldPosition.UNKNOWN){
      fieldPosition = getFieldPositionFromKeyEntry();
    }
    telemetry.addData( "FieldPosition" , fieldPosition.toString());

    // Check if we know both alliance and field position
    if (alliance != Alliance.UNKNOWN && fieldPosition != FieldPosition.UNKNOWN) {
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
      case IDENTIFY_SPIKE: {
        selectedSide = visionProcessor.getSelection();
        if (selectedSide == ScoringElementLocation.UNKNOWN) {
          break;
        }
        visionPortal.stopStreaming();
        visionPortal.setProcessorEnabled(visionProcessor, false);
        telemetry.addData("selectedSide: ", selectedSide.toString());
        state = State.PLACE_PIXEL_ON_SPIKE;
        lastTime = getRuntime();
      }
        break;
      case PLACE_PIXEL_ON_SPIKE: {
        // TODO: Depending on selectedSide, move left, straight, right.
        //  Since we are dragging the pixel there is nothing to drop.
        boolean targetReached = false;
        if (selectedSide == ScoringElementLocation.LEFT) {
          targetReached = driveToLeftSpikeMark();
        } else if (selectedSide == ScoringElementLocation.CENTER) {
          targetReached = driveToCenterSpikeMark();
        } else {
          targetReached = driveToRightSpikeMark();
        }

        if (!targetReached) {
          break;
        }

        //  Move back a little so that we don't drag the pixel away from the desired location.


        state = State.MOVE_TO_SCOREBOARD;
        lastTime = getRuntime();
      }

        break;
      case MOVE_TO_SCOREBOARD: {
        desiredTagId = getDesiredTagId(alliance, selectedSide);
        telemetry.addData("desiredTagId ", desiredTagId);
        visionPortal.setProcessorEnabled(aprilTag, true);
        // TODO: Step 1: Move based on approximation - time/ encoder / gyro.
        // Determine which direction to turn. Plan trajectory based on initial location and the goal location.
        // Keep looking for the desired aprilTag
        boolean targetFound = findDesiredAprilTag(desiredTagId);

        // Step 2: Move based on AprilTag location once it is found.
        if (!targetFound) {
          break;
        }
        boolean targetReached = driveToAprilTagTarget(desiredTag);
        if (!targetReached) {
          break;
        }
        state = State.SCORE_PIXEL;
        lastTime = getRuntime();

        visionPortal.setProcessorEnabled(aprilTag, false);
      }
        break;
      case SCORE_PIXEL: {
        // TODO: raise arm / wrist to the scoring position, open gripper
        state = State.PARK;
        lastTime = getRuntime();
      }
        break;
      case PARK: {
        // If we started on the near side, move towards the corner.
        // If we started on the far side, move to towards the center
        state = State.END;
        lastTime = getRuntime();
      }
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

  private int getDesiredTagId(Alliance alliance, ScoringElementLocation selectedSide){
    if (alliance == Alliance.UNKNOWN || selectedSide == ScoringElementLocation.UNKNOWN){
      return -1;
    }
    if (alliance == Alliance.BLUE) {
      if (selectedSide == ScoringElementLocation.LEFT) {
        return 1;
      } else if (selectedSide == ScoringElementLocation.CENTER) {
        return 2;
      } else if (selectedSide == ScoringElementLocation.RIGHT) {
        return 3;
      }
    }
    if (alliance == Alliance.RED){
      if (selectedSide == ScoringElementLocation.LEFT){
        return 4;
      } else if (selectedSide == ScoringElementLocation.CENTER) {
        return 5;
      } else if (selectedSide == ScoringElementLocation.RIGHT){
        return 6;
      }
    }

    return -1;
  }

  private Alliance getAllianceFromKeyEntry(){
    // red button
    if(gamepad1.b){
      return Alliance.RED;
    }
    if(gamepad1.x) {
      // blue button
      return Alliance.BLUE;
    }

    return Alliance.UNKNOWN;
  }


  private FieldPosition getFieldPositionFromKeyEntry(){
    // green button
    if(gamepad1.a){
      return FieldPosition.NEAR;
    }
    if(gamepad1.y){
      return FieldPosition.FAR;
    }
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
    robot.moveRobot(drive, turn);

    // TODO: Tune this
    if (rangeError < 2) {
      return true;
    }

    return false;
  }

  // TODO: Implement this
  private boolean driveToLeftSpikeMark(){
    return false;
  }

  // TODO: Implement this
  private boolean driveToCenterSpikeMark(){
    return false;
  }

  // TODO: Implement this
  private boolean driveToRightSpikeMark(){
    return false;
  }

}
