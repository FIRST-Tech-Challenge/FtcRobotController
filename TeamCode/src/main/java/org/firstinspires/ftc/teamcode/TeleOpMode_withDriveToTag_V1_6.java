package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TeleOpMode_withDriveToTag_V1_6 (Blocks to Java)")
public class TeleOpMode_withDriveToTag_V1_6 extends LinearOpMode {

  private Servo motor_dropPixels;

  double thresholdSticks;
  int stateMovingIn;
  String cameraNameFront;
  int stateMovingOut;
  double axial;
  String allianceCurrent;
  String cameraNameBack;
  int stateStopped;
  double dropPixel_stop;
  int stateOfIntake;
  double lateral;
  String cameraNameCurrent;
  int dropPixel_speed;
  double yaw;
  ElapsedTime runtime;
  int desiredTagID;
  String allianceBlue;
  double speedFactorCurrent;
  int speedFactorforHigh;
  int speedFactorforLow;
  double scalefactorStrafe;
  double scalefactorTurning;
  int desiredTagDistance;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    String allianceRed;
    boolean useVision;

    motor_dropPixels = hardwareMap.get(Servo.class, "motor_dropPixels");

    thresholdSticks = 0.2;
    cameraNameFront = "Webcam_front";
    cameraNameBack = "Webcam_back";
    cameraNameCurrent = cameraNameFront;
    allianceBlue = "Blue";
    allianceRed = "Red";
    allianceCurrent = "Blue";
    desiredTagID = 1;
    useVision = true;
    // Initialize vision libraries
    useVision = Vision.initVision(cameraNameFront, cameraNameBack);
    // Initialize all of our drivetrain motors
    runtime = new ElapsedTime();
    scalefactorTurning = 0.3;
    scalefactorStrafe = 1.5;
    speedFactorforHigh = 2;
    speedFactorforLow = 5;
    speedFactorCurrent = speedFactorforHigh;
    desiredTagDistance = 10;
    motor_dropPixels.setDirection(Servo.Direction.FORWARD);
    // Initialize Drivetrain and Pose
    DrivetrainMecanumWithSmarts.initDriveTrainWithSmarts("left_front_drive", "left_back_drive", "right_front_drive", "right_back_drive", "distance_left_front");
    // Set drive to A Config.
    DrivetrainMecanumWithSmarts.setDriveToAConfig();
    initPixelLowerEject();
    // Initialize Arm and Poses
    Arm.initArm("motor_shoulder", "motor_elbow", "motor_wrist");
    // Initialize gripper
    Gripper.init("motor_gripper", 1, 0.2, 0.5, 1);
    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    runtime.reset();
    // Run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      // Runs arm based on the state.
      Arm.runArmIteration();
      // Drive requests are by Stick or to Tags with DPad
      IfRequestingToDrive();
      IfAskedPose();
      IfAskedToggleDriveConfig();
      IfAskedToggleSpeedOrCamera();
      IfAskedDoGripper();
      IfAskedEjectLowerPixel();
      addDriveTelemetry();
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void initPixelLowerEject() {
    stateMovingIn = 1;
    stateMovingOut = -1;
    stateStopped = 0;
    stateOfIntake = stateMovingIn;
    dropPixel_speed = 10;
    dropPixel_stop = 0.5;
    motor_dropPixels.setPosition(dropPixel_stop);
  }

  /**
   * Describe this function...
   */
  private void IfAskedPose() {
    if (gamepad2.x) {
      telemetry.addLine("FUNCTION: Move to Pose X");
      // Moves the arm to an UP pose.
      Arm.moveToPose_UP();
    } else if (gamepad2.a) {
      telemetry.addLine("FUNCTION: Move to Pose FLOOR");
      // Moves the arm to an FLOOR pose.
      Arm.moveToPose_FLOOR();
    } else if (gamepad2.b) {
      telemetry.addLine("FUNCTION: Move to Pose TRAVEL");
      // Moves the arm to an TRAVEL pose.
      Arm.moveToPose_TRAVEL();
    } else if (gamepad2.y) {
      telemetry.addLine("FUNCTION: Move to Pose BACKDROP");
      // Moves the arm to an BACKDROP pose.
      Arm.moveToPose_BACKDROP();
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedDoGripper() {
    if (gamepad2.dpad_left) {
      // Opens gripper
      Gripper.GripNone();
    } else if (gamepad2.dpad_down) {
      // Grips one only
      Gripper.GripOne();
    } else if (gamepad2.dpad_right) {
      // Tightens gripper to hold two pixels
      Gripper.GripTwo();
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedEjectLowerPixel() {
    if (gamepad2.left_bumper) {
      if (stateOfIntake != stateMovingIn) {
        motor_dropPixels.setDirection(Servo.Direction.FORWARD);
        stateOfIntake = stateMovingIn;
      }
      motor_dropPixels.setPosition(dropPixel_speed);
    } else if (gamepad2.left_trigger != 0) {
      if (stateOfIntake != stateMovingOut) {
        motor_dropPixels.setDirection(Servo.Direction.REVERSE);
        stateOfIntake = stateMovingOut;
      }
      motor_dropPixels.setPosition(dropPixel_speed);
    } else {
      motor_dropPixels.setPosition(dropPixel_stop);
      stateOfIntake = stateStopped;
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedToggleSpeedOrCamera() {
    if (gamepad1.left_bumper == true) {
      // Toggle Speed
      if (speedFactorCurrent == speedFactorforHigh) {
        // Changing to low
        speedFactorCurrent = speedFactorforLow;
      } else {
        // Changing to high
        speedFactorCurrent = speedFactorforHigh;
      }
    } else if (gamepad1.left_trigger != 0) {
      // Toggle Camera
      if (cameraNameCurrent.equals(cameraNameFront)) {
        // Changing to Back camera
        // Switches the based on input.
        Vision.doCameraSwitching(cameraNameBack);
        cameraNameCurrent = cameraNameBack;
      } else {
        // Changing to Front camera
        // Switches the based on input.
        Vision.doCameraSwitching(cameraNameFront);
        cameraNameCurrent = cameraNameFront;
      }
    }
  }

  /**
   * Describe this function...
   */
  private void IfAskedToggleDriveConfig() {
    if (gamepad1.right_bumper == true) {
      // Set drive to A Config.
      DrivetrainMecanumWithSmarts.setDriveToAConfig();
    } else if (gamepad1.right_trigger != 0) {
      // Set drive to B Config.
      DrivetrainMecanumWithSmarts.setDriveToBConfig();
    }
  }

  /**
   * Describe this function...
   */
  private void IfRequestingToDrive() {
    double gainSpeed;
    double gainStrafe;
    double gainTurn;
    double maxAutoSpeed;
    double maxAutoStrafe;
    double maxAutoTurn;
    double tagRange;
    double tagBearing;
    double tagYaw;
    double desiredTagRange;

    if (Math.abs(gamepad1.left_stick_y) >= thresholdSticks || Math.abs(gamepad1.left_stick_x) >= thresholdSticks || Math.abs(gamepad1.right_stick_x) >= thresholdSticks) {
      // Driver is driving based on Sticks
      // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
      // Note: pushing stick forward gives negative value
      axial = -(gamepad1.left_stick_y / speedFactorCurrent);
      lateral = (gamepad1.left_stick_x / speedFactorCurrent) * scalefactorStrafe;
      yaw = gamepad1.right_stick_x * scalefactorTurning;
      addDriveTelemetry();
      // Moves based on axial(Y,forward/backward), lateral(X, side-to-side) and yaw (turning)
      DrivetrainMecanumWithSmarts.DriveXYT(axial, lateral, yaw);
    } else if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
      // Driver asking to drive to an AprilTag target
      gainSpeed = 0.02;
      gainStrafe = 0.015;
      gainTurn = 0.01;
      maxAutoSpeed = 0.05;
      maxAutoStrafe = 0.05;
      maxAutoTurn = 0.03;
      if (gamepad1.dpad_left) {
        if (allianceCurrent.equals(allianceBlue)) {
          desiredTagID = 1;
        } else {
          desiredTagID = 1 + 3;
        }
      } else if (gamepad1.dpad_up) {
        if (allianceCurrent.equals(allianceBlue)) {
          desiredTagID = 2;
        } else {
          desiredTagID = 2 + 3;
        }
      } else if (gamepad1.dpad_right) {
        if (allianceCurrent.equals(allianceBlue)) {
          desiredTagID = 3;
        } else {
          desiredTagID = 3 + 3;
        }
      } else if (gamepad1.dpad_down) {
      }
      // Provides an AprilTagDetection structure for the detected April Tag.
      if (Vision.getTagInfo(desiredTagID)) {
        // Returns the Range only
        tagRange = Vision.getTagRange(desiredTagID);
        // Returns the Bearing only
        tagBearing = Vision.getTagBearing(desiredTagID);
        // Returns the Yaw only
        tagYaw = Vision.getTagYaw(desiredTagID);
        // Forward amount
        desiredTagRange = tagRange - desiredTagDistance;
        axial = Math.min(Math.max(desiredTagRange * gainSpeed, -maxAutoSpeed), maxAutoSpeed);
        // Strafe amount
        lateral = Math.min(Math.max(-tagYaw * gainStrafe, -maxAutoStrafe), maxAutoStrafe);
        // Turn amount
        yaw = Math.min(Math.max(tagBearing * gainTurn, -maxAutoTurn), maxAutoTurn);
        addDriveTelemetry();
        // Moves based on axial(Y,forward/backward), lateral(X, side-to-side) and yaw (turning)
        DrivetrainMecanumWithSmarts.DriveXYT(axial, lateral, yaw);
      } else {
        axial = 0;
        lateral = 0;
        yaw = 0;
      }
    } else {
      axial = 0;
      lateral = 0;
      yaw = 0;
    }
  }

  /**
   * Describe this function...
   */
  private void addDriveTelemetry() {
    telemetry.addData("ALLIANCE", allianceCurrent);
    telemetry.addData("Status", "Run Time: " + runtime);
    telemetry.addData("DESIRED TAG ID", desiredTagID);
    telemetry.addData("AXIAL, LATERAL, YAW", JavaUtil.formatNumber(axial, 4, 2) + ", " + JavaUtil.formatNumber(lateral, 4, 2) + ", " + JavaUtil.formatNumber(yaw, 4, 2));
  }

  /**
   * This function is used to test your motor directions.
   */
  private void testMotorDirections() {
    // TODO: Enter the type for variable named leftFrontPower
    UNKNOWN_TYPE leftFrontPower;
    // TODO: Enter the type for variable named leftBackPower
    UNKNOWN_TYPE leftBackPower;
    // TODO: Enter the type for variable named rightFrontPower
    UNKNOWN_TYPE rightFrontPower;
    // TODO: Enter the type for variable named rightBackPower
    UNKNOWN_TYPE rightBackPower;

    // Each button should make the corresponding motor run FORWARD.
    //   1) First get all the motors to take to correct positions on the robot
    //      by adjusting your Robot Configuration if necessary.
    //   2) Then make sure they run in the correct direction by modifying the
    //      the setDirection() calls above.
    leftFrontPower = gamepad1.x ? 1 : 0;
    leftBackPower = gamepad1.a ? 1 : 0;
    rightFrontPower = gamepad1.y ? 1 : 0;
    rightBackPower = gamepad1.b ? 1 : 0;
  }
}
