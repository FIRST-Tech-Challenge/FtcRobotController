package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto_DropThenBackdrop_v1_4 (Blocks to Java)")
public class Auto_DropThenBackdrop_v1_4 extends LinearOpMode {

  private Servo motor_dropPixels;
  private Servo motor_shoulder;

  int DropPixels_Go;
  double DropPixels_Stop;
  int spikeTarget;
  int spikeRight;
  int spikeLeft;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    String allianceRed;
    String allianceBlue;
    String allianceCurrent;
    String locationBackstage;
    String locationAudience;
    String locationCurrent;
    int spikeCenter;
    int minXforSpikeRight;
    boolean useVision;
    int distanceToMid;
    double yawToRightSpike;
    double yawToLeftSpike;
    int distanceToSpike;

    motor_dropPixels = hardwareMap.get(Servo.class, "motor_dropPixels");
    motor_shoulder = hardwareMap.get(Servo.class, "motor_shoulder");

    allianceRed = "Red";
    allianceBlue = "Blue";
    allianceCurrent = allianceBlue;
    locationBackstage = "Backstage";
    locationAudience = "Audience";
    locationCurrent = locationBackstage;
    // Helps us determine the spike marks
    spikeLeft = 1;
    spikeCenter = 2;
    spikeRight = 3;
    // Using pixels on image to determine which spike mark the object is on
    minXforSpikeRight = 300;
    DropPixels_Stop = 0.5;
    DropPixels_Go = 10;
    motor_dropPixels.setDirection(Servo.Direction.FORWARD);
    motor_dropPixels.setPosition(DropPixels_Stop);
    // Initialize our vision
    // Initialize vision libraries
    useVision = Vision.initVision("Webcam_front", "Webcam_back");
    // Initialize Drivetrain and Pose
    DrivetrainMecanumWithSmarts.initDriveTrainWithSmarts("left_front_drive", "left_back_drive", "right_front_drive", "right_back_drive", "distance_left_front");
    // Initialize Arm and Poses
    Arm.initArm("motor_shoulder", "motor_elbow", "motor_wrist");
    // Moving the shoulder up slightly to allow lower pixels to move freely
    motor_shoulder.setPosition(0.4);
    // Initialize gripper
    Gripper.init("motor_gripper", 1, 0.2, 0.5, 1);
    // Distance to be equal distance in the middle of all three spikes
    distanceToMid = 50;
    // Yaw to turn clockwise (to right spike) or counterclockwise (to left spike)
    yawToRightSpike = 0.5;
    yawToLeftSpike = -0.5;
    // Distance to the spike.
    distanceToSpike = 75;
    telemetry.addData("ALLIANCE", allianceCurrent);
    // Wait for the match to begin.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();
    spikeTarget = spikeLeft;
    while (opModeInInit()) {
      if (useVision) {
        telemetry.addData("USE VISION", "true");
        // Indicates whether the Team Prop is in sight.
        spikeTarget = Vision.getTeamPropLocation("Bolt", minXforSpikeRight);
      } else {
        telemetry.addData("USE VISION", "false");
      }
      if (gamepad1.a) {
        locationCurrent = locationAudience;
        telemetry.addData("LOCATION", "Audience");
      } else if (gamepad1.b) {
        locationCurrent = locationBackstage;
      } else if (gamepad1.x) {
        allianceCurrent = allianceBlue;
      } else if (gamepad1.y) {
        allianceCurrent = allianceRed;
      }
      telemetry.addData("ALLIANCE", allianceCurrent);
      telemetry.addData("Target spike (1=Left, 2=Center, 3=Right)", spikeTarget);
      telemetry.update();
    }
    telemetry.addData("Target spike (1=Left, 2=Center, 3=Right)", spikeTarget);
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      telemetry.addData("Target spike (1=Left, 2=Center, 3=Right)", spikeTarget);
      telemetry.update();
      // Make sure our drivetrain is waiting for a command
      // Returns whether we are waiting for a command.
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand() && Arm.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        // Runs arm based on the state.
        Arm.runArmIteration();
        telemetry.update();
      }
      // Ready to perform first goal of placing purple pixel on correct spike mark
      requestToMoveAndDropPurplePixel();
      // Ready to perform second goal of scoring yellow pixel above correct AprilTag
      // Ready to perform last goal of parking
      parkFromSpikeTileToBackstageWallTile();
    }
  }

  /**
   * Describe this function...
   */
  private void DropPixel() {
    motor_dropPixels.setPosition(DropPixels_Go);
    sleep(4000);
    motor_dropPixels.setPosition(DropPixels_Stop);
  }

  /**
   * Describe this function...
   */
  private void requestToMoveAndDropPurplePixel() {
    // Let's move to the middle of the spike tile
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, 22, 0);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    // Turn to left or right spike if necessary
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      if (spikeTarget == spikeRight) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        DrivetrainMecanumWithSmarts.turnToHeading(0.75, -50);
        // Returns whether we are waiting for a command.
        while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
          // Runs drivetrain based on the state.
          DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        }
      } else if (spikeTarget == spikeLeft) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        DrivetrainMecanumWithSmarts.turnToHeading(0.75, 50);
        // Returns whether we are waiting for a command.
        while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
          // Runs drivetrain based on the state.
          DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        }
      }
    }
    // Move to spike
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, 7, 0);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    telemetry.update();
    if (opModeIsActive()) {
      DropPixel();
    }
    // Move away spike
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, -7, 0);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    // Turn to left or right if necessary
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      if (spikeTarget == spikeRight) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        DrivetrainMecanumWithSmarts.turnToHeading(0.5, 0);
        // Returns whether we are waiting for a command.
        while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
          // Runs drivetrain based on the state.
          DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        }
      } else if (spikeTarget == spikeLeft) {
        // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
        DrivetrainMecanumWithSmarts.turnToHeading(0.5, 0);
        // Returns whether we are waiting for a command.
        while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
          // Runs drivetrain based on the state.
          DrivetrainMecanumWithSmarts.runDrivetrainIteration();
        }
      }
    }
    // Move back away from spikes
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, -7, 0);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void parkFromSpikeTileToBackstageWallTile() {
    // Turn to face backdrop AprilTags
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.
      DrivetrainMecanumWithSmarts.turnToHeading(0.5, 90);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
    }
    // Move left towards wall
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Left or Right.
      DrivetrainMecanumWithSmarts.driveLeft(0.3, 16, 90);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    // Move to backstage
    // Returns whether we are waiting for a command.
    if (opModeIsActive() && DrivetrainMecanumWithSmarts.waitingForCommand()) {
      // Drives Straight either Forward or Reverse.
      DrivetrainMecanumWithSmarts.driveStraight(0.3, 40, 90);
      // Returns whether we are waiting for a command.
      while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand()) {
        // Runs drivetrain based on the state.
        DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      }
      telemetry.update();
    }
    if (opModeIsActive()) {
      DropPixel();
    }
    motor_shoulder.setPosition(0.3);
    sleep(4000);
  }

  /**
   * Describe this function...
   */
  private void requestToMoveToBackdropAndDropYellowPixel() {
    // Returns whether we are waiting for a command.
    // Returns whether we are waiting for a command.
    while (opModeIsActive() && !DrivetrainMecanumWithSmarts.waitingForCommand() && Arm.waitingForCommand()) {
      // Runs drivetrain based on the state.
      DrivetrainMecanumWithSmarts.runDrivetrainIteration();
      // Runs arm based on the state.
      Arm.runArmIteration();
      telemetry.update();
    }
  }
}
