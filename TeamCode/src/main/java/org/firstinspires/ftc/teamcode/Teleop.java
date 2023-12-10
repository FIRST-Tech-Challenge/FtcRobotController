/* FTC Team 7572 - Version 1.0 (11/10/2023)
*/
package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * TeleOp Full Control.
 */
//@Disabled
public abstract class Teleop extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;  // 
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;  // 
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;  // 
    boolean gamepad1_square_last,     gamepad1_square_now     = false;  // 
    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;  // gamepad1.dpad_up used live/realtime
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;  //   (see processDpadDriveMode() below)
    boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;
    boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;
    boolean gamepad1_touchpad_last,   gamepad1_touchpad_now   = false;  
    boolean gamepad1_l_trigger_last,  gamepad1_l_trigger_now  = false;
    boolean gamepad1_r_trigger_last,  gamepad1_r_trigger_now  = false;

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  // 
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  // 
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  // 
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  // 
    boolean gamepad2_dpad_up_last,    gamepad2_dpad_up_now    = false;  // 
    boolean gamepad2_dpad_down_last,  gamepad2_dpad_down_now  = false;  // 
    boolean gamepad2_dpad_left_last,  gamepad2_dpad_left_now  = false;  // 
    boolean gamepad2_dpad_right_last, gamepad2_dpad_right_now = false;  // 
    boolean gamepad2_l_bumper_last,   gamepad2_l_bumper_now   = false;  // 
    boolean gamepad2_r_bumper_last,   gamepad2_r_bumper_now   = false;  // 
    boolean gamepad2_touchpad_last,   gamepad2_touchpad_now   = false;  // 
    boolean gamepad2_share_last,      gamepad2_share_now      = false;  // 

    double  yTranslation, xTranslation, rotation;                  /* Driver control inputs */
    double  rearLeft, rearRight, frontLeft, frontRight, maxPower;  /* Motor power levels */
    boolean backwardDriveControl = false; // drive controls backward (other end of robot becomes "FRONT")
    boolean controlMultSegLinear = true;

    final int DRIVER_MODE_STANDARD     = 2;
    final int DRIVER_MODE_DRV_CENTRIC  = 3;
    int       driverMode               = DRIVER_MODE_STANDARD;
//  int       driverMode               = DRIVER_MODE_DRV_CENTRIC;
    double    driverAngle              = 0.0;  /* for DRIVER_MODE_DRV_CENTRIC */

    boolean   batteryVoltsEnabled = false;  // enable only during testing (takes time!)

    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    elapsedTime, elapsedHz;

    /* Declare OpMode members. */
    HardwarePixelbot robot = new HardwarePixelbot();

    //Files to access the algorithm constants
    File wheelBaseSeparationFile  = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    double robotEncoderWheelDistance            = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * robot.COUNTS_PER_INCH2;
    double horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    double robotGlobalXCoordinatePosition       = 0.0;   // in odometer counts
    double robotGlobalYCoordinatePosition       = 0.0;
    double robotOrientationRadians              = 0.0;   // 0deg (straight forward)

    boolean leftAlliance = true;  // overriden in setAllianceSpecificBehavior()
    int     aprilTagLeft   = 1;   // overriden in setAllianceSpecificBehavior()
    int     aprilTagCenter = 2;   // overriden in setAllianceSpecificBehavior()
    int     aprilTagRight  = 3;   // overriden in setAllianceSpecificBehavior()

    boolean liftTweaked  = false;  // Reminder to zero power when input stops

    Gamepad.RumbleEffect visibleAprilTagRumble1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect visibleAprilTagRumble2;    // Use to build a custom rumble sequence.

//========== DRIVE-TO-APRILTAG variables ==========

    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

//========== DRIVE-TO-APRILTAG variables ==========

//  double elbowPos = 0.500;
//  double wristPos = 0.500;
//  double collPos  = 0.500;

    // LIFT STATE MACHINE VARIABLES
    final int LIFT_STATE_IDLE = 0;
    final int LIFT_STATE_ELBOW_INTO_BIN = 1;  // ready to rotate elbow into the pixel bin
    final int LIFT_STATE_FINGERS_GRAB_PIXELS = 2;  // ready to open fingers to grab pixels
    final int LIFT_STATE_LIFT_TO_SCORE = 3; // ready to raise the lift to the scoring position
    final int LIFT_STATE_WRIST_TO_SCORE = 4; // ready to rotate wrist to the scoring position
    final int LIFT_STATE_FINGERS_DROP_TO_SCORE = 4; // ready to rotate wrist to the scoring position

    int       liftState = LIFT_STATE_IDLE;

    // sets unique behavior based on alliance
    public abstract void setAllianceSpecificBehavior();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        visibleAprilTagRumble1 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble left/right motors 100% for 500 mSec
                .build();
        visibleAprilTagRumble2 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 500 mSec
                .build();

        // Initialize robot hardware
        robot.init(hardwareMap,false);

        setAllianceSpecificBehavior();

        // Initialize the Apriltag Detection process
        initAprilTag();
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

       // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Refresh gamepad button status
            captureGamepad1Buttons();
            captureGamepad2Buttons();

            // Bulk-refresh the Control/Expansion Hub device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();
            globalCoordinatePositionUpdate();

           //ProcessAprilTagControls();
            ProcessCollectorControls();
            ProcessFingerControls();
            ProcessLiftControls();
           //ProcessLiftStateMachine();

            // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button (toggles DRIVER-CENTRIC drive control)
            if( gamepad1_square_now && !gamepad1_square_last)
            {
                driverMode = DRIVER_MODE_DRV_CENTRIC;
            }

            // Check for an OFF-to-ON toggle of the gamepad1 CIRCLE button (toggles STANDARD/BACKWARD drive control)
            if( gamepad1_circle_now && !gamepad1_circle_last)
            {
                // If currently in DRIVER-CENTRIC mode, switch to STANDARD (robot-centric) mode
                if( driverMode != DRIVER_MODE_STANDARD ) {
                    driverMode = DRIVER_MODE_STANDARD;
                    backwardDriveControl = true;  // start with phone-end as front of robot
                }
                // Already in STANDARD mode; Just toggle forward/backward mode
                else {
                    backwardDriveControl = !backwardDriveControl; // reverses which end of robot is "FRONT"
                }
            }

//            telemetry.addData("circle","Robot-centric (fwd/back modes)");
//            telemetry.addData("square","Driver-centric (set joystick!)");
//            telemetry.addData("d-pad","Fine control (30%)");
//            telemetry.addData(" "," ");

//            telemetry.addData("Elbow", "%.3f", elbowPos );
//            telemetry.addData("Wrist", "%.3f", wristPos );
//            telemetry.addData("Coll", "%.3f", collPos);

            if( processDpadDriveMode() == false ) {
                // Control based on joystick; report the sensed values
                telemetry.addData("Joystick", "x=%.3f, y=%.3f spin=%.3f",
                        -gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x );
                switch( driverMode ) {
                    case DRIVER_MODE_STANDARD :
                        telemetry.addData("Driver Mode", "STD-%s (cir)",
                                (backwardDriveControl)? "BACKWARD":"FORWARD" );
                        processStandardDriveMode();
                        break;
                    case DRIVER_MODE_DRV_CENTRIC :
                        telemetry.addData("Driver Mode", "DRIVER-CENTRIC (sq)" );
                        processDriverCentricDriveMode();
                        break;
                    default :
                        // should never happen; reset to standard drive mode
                        driverMode = DRIVER_MODE_STANDARD;
                        break;
                } // switch()
            } // processDpadDriveMode

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            elapsedTime  = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            elapsedHz    =  1000.0 / elapsedTime;

            // Update telemetry data
            telemetry.addData("Servo", "%.3f (%d)", robot.collectorServoSetPoint, robot.collectorServoIndex);
            telemetry.addData("Viper", "%d cts (%.2f mA)", robot.viperMotorsPos, robot.viperMotorsPwr );
            telemetry.addData("Front", "%.2f (%d cts) %.2f (%d cts)",
                    frontLeft, robot.frontLeftMotorPos, frontRight, robot.frontRightMotorPos );
            telemetry.addData("Rear ", "%.2f (%d cts) %.2f (%d cts)",
                    rearLeft,  robot.rearLeftMotorPos,  rearRight,  robot.rearRightMotorPos );
            telemetry.addData("Odometry (L/R/S)", "%d %d %d cts",
                    robot.leftOdometerCount, robot.rightOdometerCount, robot.strafeOdometerCount );
            telemetry.addData("World X",     "%.2f in", (robotGlobalYCoordinatePosition / robot.COUNTS_PER_INCH2) );
            telemetry.addData("World Y",     "%.2f in", (robotGlobalXCoordinatePosition / robot.COUNTS_PER_INCH2) );
            telemetry.addData("Gyro Angle", "%.1f degrees", robot.headingIMU() );
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", elapsedTime, elapsedHz );
            telemetry.addData("RightTrigger", "%.2f", gamepad2.right_trigger);
            if( batteryVoltsEnabled ) {
               telemetry.addData("Batteries", "CtlHub=%.3f V, ExHub=%.3f V",
                    robot.readBatteryControlHub()/1000.0, robot.readBatteryExpansionHub()/1000.0 );
            }
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//          robot.waitForTick(40);
        } // opModeIsActive

    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void captureGamepad1Buttons() {
        gamepad1_triangle_last   = gamepad1_triangle_now;    gamepad1_triangle_now   = gamepad1.triangle;
        gamepad1_circle_last     = gamepad1_circle_now;      gamepad1_circle_now     = gamepad1.circle;
        gamepad1_cross_last      = gamepad1_cross_now;       gamepad1_cross_now      = gamepad1.cross;
        gamepad1_square_last     = gamepad1_square_now;      gamepad1_square_now     = gamepad1.square;
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
        gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
        gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
//      gamepad1_touchpad_last   = gamepad1_touchpad_now;    gamepad1_touchpad_now   = gamepad1.touchpad;
        gamepad1_l_trigger_last  = gamepad1_l_trigger_now;   gamepad1_l_trigger_now  = (gamepad1.left_trigger >= 0.5);
        gamepad1_r_trigger_last  = gamepad1_r_trigger_now;   gamepad1_r_trigger_now  = (gamepad1.right_trigger >= 0.5);
    } // captureGamepad1Buttons

    /*---------------------------------------------------------------------------------*/
    void captureGamepad2Buttons() {
        gamepad2_triangle_last   = gamepad2_triangle_now;    gamepad2_triangle_now   = gamepad2.triangle;
        gamepad2_circle_last     = gamepad2_circle_now;      gamepad2_circle_now     = gamepad2.circle;
        gamepad2_cross_last      = gamepad2_cross_now;       gamepad2_cross_now      = gamepad2.cross;
        gamepad2_square_last     = gamepad2_square_now;      gamepad2_square_now     = gamepad2.square;
        gamepad2_dpad_up_last    = gamepad2_dpad_up_now;     gamepad2_dpad_up_now    = gamepad2.dpad_up;
        gamepad2_dpad_down_last  = gamepad2_dpad_down_now;   gamepad2_dpad_down_now  = gamepad2.dpad_down;
        gamepad2_dpad_left_last  = gamepad2_dpad_left_now;   gamepad2_dpad_left_now  = gamepad2.dpad_left;
        gamepad2_dpad_right_last = gamepad2_dpad_right_now;  gamepad2_dpad_right_now = gamepad2.dpad_right;
        gamepad2_l_bumper_last   = gamepad2_l_bumper_now;    gamepad2_l_bumper_now   = gamepad2.left_bumper;
        gamepad2_r_bumper_last   = gamepad2_r_bumper_now;    gamepad2_r_bumper_now   = gamepad2.right_bumper;
  //    gamepad2_touchpad_last   = gamepad2_touchpad_now;    gamepad2_touchpad_now   = gamepad2.touchpad;
  //    gamepad2_share_last      = gamepad2_share_now;       gamepad2_share_now      = gamepad2.share;
    } // captureGamepad2Buttons

    /*---------------------------------------------------------------------------------*/
    void ProcessAprilTagControls() {
        
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  minDrvPwr       = 0.05;     // minimum power needed to drive robot forward
        double  minStrafePwr    = 0.12;
        double  minTurnPwr      = 0.06;
        double  driveErr        = 0.0;
        double  strafeErr       = 0.0;
        double  turnErr         = 0.0;
        double  drive           = 0.0;      // Desired forward power/speed (-1 to +1)
        double  strafe          = 0.0;      // Desired strafe power/speed (-1 to +1)
        double  turn            = 0.0;      // Desired turning power/speed (-1 to +1)

        boolean hasRumbled = false;
        
        AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;  //  Forward Speed Control "Gain". eg: Ramp up to 75% power at a 25 inch error.   (0.75 / 25.0)
    final double STRAFE_GAIN =  0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.75;  //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.50;  //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.30;  //  Clip the turn speed to this max value (adjust for your robot)

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
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

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                if(!hasRumbled) {
                    gamepad1.runRumbleEffect(visibleAprilTagRumble1);
                    hasRumbled = true;
                }
                telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                hasRumbled = false;
                telemetry.addData(">","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError = desiredTag.ftcPose.bearing;
                double  yawError     = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                driveErr  = (Math.abs(rangeError) < 0.2)? 0.0 : (minDrvPwr + rangeError * SPEED_GAIN);
                turnErr   = (Math.abs(headingError) < 0.4)? 0.0 : (minTurnPwr + headingError * TURN_GAIN);
                strafeErr = (Math.abs(yawError) < 0.2)? 0.0 : (minStrafePwr - yawError * STRAFE_GAIN);
                drive  = Range.clip( driveErr, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip( turnErr, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip( strafeErr, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                if( Math.abs(drive)  < 0.03 ) drive  = 0.0;
                if( Math.abs(strafe) < 0.03 ) strafe = 0.0;
                if( Math.abs(turn)   < 0.05 ) turn   = 0.0;
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

    } // ProcessAprilTagControls
    
    /*---------------------------------------------------------------------------------*/
    void ProcessCollectorControls() {
        // Check for an OFF-to-ON toggle of the gamepad2 CROSS button
        // - turns off collector motor
        // - raises collector for driving
        if( gamepad2_cross_now && !gamepad2_cross_last)
        {
            robot.collectorMotor.setPower(0.0);
            robot.collectorServo.setPosition(robot.COLLECTOR_SERVO_RAISED);
        }
        // Check for an OFF-to-ON toggle of the gamepad2 left or right bumpers
        // - left enables the collector motor in REVERSE mode
        // - right enables the collector motor in FORWARD mode
        if( gamepad2_l_bumper_now && !gamepad2_l_bumper_last)
        {
          robot.collectorMotor.setPower(robot.COLLECTOR_MOTOR_POWER);
//        elbowPos -= 0.01;
//        robot.elbowServo.setPosition(elbowPos);
//        wristPos -= 0.01;
//        robot.wristServo.setPosition(wristPos);
//        collPos -= 0.01;
//        robot.collectorServo.setPosition(collPos);
        }
        else if( gamepad2_r_bumper_now && !gamepad2_r_bumper_last)
        {
          robot.collectorMotor.setPower(-robot.COLLECTOR_MOTOR_POWER);
//        elbowPos += 0.01;
//        robot.elbowServo.setPosition(elbowPos);
//        wristPos += 0.01;
//        robot.wristServo.setPosition(wristPos);
//        collPos += 0.01;
//        robot.collectorServo.setPosition(collPos);
        }
        // Check for an OFF-to-ON toggle of the gamepad2 CIRCLE button
        // - lowers collector for grabbing pixels
        // - turns on the collector motor in FORWARD mode
        if( gamepad2_circle_now && !gamepad2_circle_last)
        {
            robot.collectorServo.setPosition(robot.COLLECTOR_SERVO_GROUND);
            robot.collectorMotor.setPower(-robot.COLLECTOR_MOTOR_POWER);
        }

    }  // processCollectorControls


    /*---------------------------------------------------------------------------------*/
    void ProcessFingerControls() {

        // Check for an OFF-to-ON toggle of the gamepad2 TRIANGLE button
        if( gamepad2_triangle_now && !gamepad2_triangle_last)
        {
            // Make sure we're lifted before we allow the operator to command SCORE
            if( robot.viperMotorsPos > robot.VIPER_EXTEND_BIN ) {
                robot.elbowServo.setPosition(robot.ELBOW_SERVO_DROP);
                robot.wristServo.setPosition(robot.WRIST_SERVO_DROP);
                // Wait 2.5 seconds (1 sec to rotate the servos, plus 1.5 sec more to stop wobbling
                // so we don't end up "flinging" the pixel against the backdrop.
                sleep(2500 );
                robot.fingerServo1.setPosition(robot.FINGER1_SERVO_DROP);
                robot.fingerServo2.setPosition(robot.FINGER2_SERVO_DROP);
            }
        }

        // Check for an OFF-to-ON toggle of the gamepad2 SQUARE button
        else if( gamepad2_square_now && !gamepad2_square_last)
        {
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_GRAB);
            robot.wristServo.setPosition(robot.WRIST_SERVO_GRAB);
            sleep(1000);
            robot.fingerServo1.setPosition(robot.FINGER1_SERVO_GRAB);
            robot.fingerServo2.setPosition(robot.FINGER2_SERVO_GRAB);
        }

    } // ProcessFingerControls

    /*---------------------------------------------------------------------------------*/
    void ProcessLiftControls() {
        boolean safeToManuallyLower = (robot.viperMotorsPos > robot.VIPER_EXTEND_ZERO);
        boolean safeToManuallyRaise = (robot.viperMotorsPos < robot.VIPER_EXTEND_FULL);
        // Capture user inputs ONCE, in case they change during processing of this code
        // or we want to scale them down
        double  gamepad2_left_trigger  = gamepad2.left_trigger  * 1.00;
        double  gamepad2_right_trigger = gamepad2.right_trigger * 1.00;
        boolean manual_lift_control = ( (gamepad2_left_trigger  > 0.25) ||
                                        (gamepad2_right_trigger > 0.25) );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD UP
        if( gamepad2_dpad_up_now && !gamepad2_dpad_up_last)
        {   // Move lift to HIGH-SCORING position
           robot.startViperSlideExtension( robot.VIPER_EXTEND_HIGH );
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD RIGHT
        else if( gamepad2_dpad_right_now && !gamepad2_dpad_right_last)
        {   // Move lift to MID-SCORING position
           robot.startViperSlideExtension( robot.VIPER_EXTEND_MID );
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD DOWN
        else if( gamepad2_dpad_down_now && !gamepad2_dpad_down_last)
        {   // Move lift to LOW-SCORING position
           robot.startViperSlideExtension( robot.VIPER_EXTEND_LOW );
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD LEFT
        else if( gamepad2_dpad_left_now && !gamepad2_dpad_left_last)
        {   // Move lift to STORED position
           robot.elbowServo.setPosition(robot.ELBOW_SERVO_SAFE);
           robot.wristServo.setPosition(robot.WRIST_SERVO_GRAB);
           sleep(750);
           robot.startViperSlideExtension( robot.VIPER_EXTEND_ZERO );
        }
        //===================================================================
        else if( manual_lift_control || liftTweaked ) {
            // Does user want to manually RAISE the lift?
            if( safeToManuallyRaise && (gamepad2_right_trigger > 0.25) ) {
                // Do we need to terminate an auto movement?
                robot.checkViperSlideExtension();
                robot.viperMotors.setPower( gamepad2_right_trigger );  // fixed power? (robot.VIPER_RAISE_POWER)
                liftTweaked = true;
            }
            // Does user want to manually LOWER the lift?
            else if( safeToManuallyLower && (gamepad2_left_trigger > 0.25) ) {
                // Do we need to terminate an auto movement?
                robot.checkViperSlideExtension();
                robot.viperMotors.setPower( robot.VIPER_LOWER_POWER );
                liftTweaked = true;
            }
            // No more input?  Time to stop lift movement!
            else if( liftTweaked ) {
                // if the lift is near the bottom, truly go to zero power
                // but if in a raised position, only drop to minimal holding power
                boolean closeToZero = (Math.abs(robot.viperMotorsPos - robot.VIPER_EXTEND_ZERO) < 20);
                robot.viperMotors.setPower( closeToZero? 0.0 : robot.VIPER_HOLD_POWER );
                liftTweaked = false;
            }
        } // manual_lift_control

    }  // ProcessLiftControls

    /*---------------------------------------------------------------------------------*/
    void ProcessLiftStateMachine() {

        if( liftState != LIFT_STATE_IDLE ) {

        }


    } // ProcessLiftStateMachine

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Mecanum-wheel drive control using Dpad (slow/fine-adjustment mode)    */
    /*---------------------------------------------------------------------------------*/
    boolean processDpadDriveMode() {
        double fineDriveSpeed  = 0.50;
        double fineStrafeSpeed = 0.50;
        double autoDriveSpeed  = 0.56;
        double fineTurnSpeed   = 0.05;
        boolean dPadMode = true;
        // Only process 1 Dpad button at a time
        if( gamepad1.dpad_up ) {
            telemetry.addData("Dpad","FORWARD");
            frontLeft  = fineDriveSpeed;
            frontRight = fineDriveSpeed;
            rearLeft   = fineDriveSpeed;
            rearRight  = fineDriveSpeed;
        }
        else if( gamepad1.dpad_down ) {
            telemetry.addData("Dpad","BACKWARD");
            frontLeft  = -fineDriveSpeed;
            frontRight = -fineDriveSpeed;
            rearLeft   = -fineDriveSpeed;
            rearRight  = -fineDriveSpeed;
        }
        else if( gamepad1.dpad_left ) {
            telemetry.addData("Dpad","LEFT");
            frontLeft  = -fineStrafeSpeed;
            frontRight =  fineStrafeSpeed;
            rearLeft   =  fineStrafeSpeed;
            rearRight  = -fineStrafeSpeed;
        }
        else if( gamepad1.dpad_right ) {
            telemetry.addData("Dpad","RIGHT");
            frontLeft  =  fineStrafeSpeed;
            frontRight = -fineStrafeSpeed;
            rearLeft   = -fineStrafeSpeed;
            rearRight  =  fineStrafeSpeed;
        }

/*  INSTEAD USE LEFT/RIGHT FOR FINE-TURNING CONTROL
        else if( gamepad1.dpad_left ) {
            telemetry.addData("Dpad","TURN");
            frontLeft  = -fineTurnSpeed;
            frontRight =  fineTurnSpeed;
            rearLeft   = -fineTurnSpeed;
            rearRight  =  fineTurnSpeed;
        }
        else if( gamepad1.dpad_right ) {
            telemetry.addData("Dpad","TURN");
            frontLeft  =  fineTurnSpeed;
            frontRight = -fineTurnSpeed;
            rearLeft   =  fineTurnSpeed;
            rearRight  = -fineTurnSpeed;
        }
 */

 /* TOUCHPAD CONTROL FOR AUTO-DRIVE NOT USED FOR TELEOP THIS YEAR
       else if( autoDrive || (gamepad1_touchpad_now && !gamepad1_touchpad_last) ) {
            telemetry.addData("Touchpad","FORWARD");
            frontLeft  = autoDriveSpeed;
            frontRight = autoDriveSpeed;
            rearLeft   = autoDriveSpeed;
            rearRight  = autoDriveSpeed;
            autoDrive = true;
        }
  */
        else {
            dPadMode = false;
        }
        if( dPadMode ) {
            robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight);
        }
        return dPadMode;
    } // processDpadDriveMode

    private double minThreshold( double valueIn ) {
        double valueOut;

        //========= NO/MINIMAL JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.02 ) {
            valueOut = 0.0;
        }
        else {
            valueOut = valueIn;
        }
        return valueOut;
    } // minThreshold

    private double multSegLinearRot( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.05 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.33 ) {                      // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.0650;   // 0.02=0.070  0.33=0.1475
            }
            else if( valueIn < 0.60 ) {
                valueOut = (0.50 * valueIn) - 0.0175;   // 0.33=0.1475  0.60=0.2825
            }
            else if( valueIn < 0.90 ) {
                valueOut = (0.75 * valueIn) - 0.1675;   // 0.60=0.2825  0.90=0.5075
            }
            else
                valueOut = (6.00 * valueIn) - 4.8925;   // 0.90=0.5075  1.00=1.1075 (clipped!)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.33 ) {
                valueOut = (0.25 * valueIn) - 0.0650;
            }
            else if( valueIn > -0.60 ) {
                valueOut = (0.50 * valueIn) + 0.0175;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (0.75 * valueIn) + 0.1675;
            }
            else
                valueOut = (6.00 * valueIn) + 4.8925;
        }

        return valueOut/2.0;
    } // multSegLinearRot

    private double multSegLinearXY( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.05 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.50 ) {                       // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.040;     // 0.01=0.0425   0.50=0.1650
            }
            else if( valueIn < 0.90 ) {
                valueOut = (0.75 * valueIn) - 0.210;     // 0.50=0.1650   0.90=0.4650
            }
            else
                valueOut = (8.0 * valueIn) - 6.735;      // 0.90=0.4650   1.00=1.265 (clipped)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.50 ) {
                valueOut = (0.25 * valueIn) - 0.040;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (0.75 * valueIn) + 0.210;
            }
            else
                valueOut = (8.0 * valueIn) + 6.735;
        }

        return valueOut;
    } // multSegLinearXY

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Drive-motor diagnostic tool (command one wheel/motor at a time)       */
    /*---------------------------------------------------------------------------------*/
    void processSingleWheelControl() {
        // Use the motor-power variables so our telemetry updates correctly
        frontLeft  = minThreshold( gamepad1.left_stick_y  );
        frontRight = minThreshold( gamepad1.right_stick_y );
        rearLeft   = minThreshold( gamepad1.left_stick_x  );
        rearRight  = minThreshold( gamepad1.right_stick_x );

        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );
    } // processSingleWheelControl

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Standard Mecanum-wheel drive control (no dependence on gyro!)         */
    /*---------------------------------------------------------------------------------*/
    void processStandardDriveMode() {
        // Retrieve X/Y and ROTATION joystick input
        if( controlMultSegLinear ) {
            yTranslation = multSegLinearXY( -gamepad1.left_stick_y );
            xTranslation = multSegLinearXY(  gamepad1.left_stick_x );
            rotation     = multSegLinearRot( -gamepad1.right_stick_x );
        }
        else {
            yTranslation = -gamepad1.left_stick_y * 1.00;
            xTranslation =  gamepad1.left_stick_x * 1.25;
            rotation     = -gamepad1.right_stick_x * 0.50;
        }
        // If BACKWARD drive control, reverse the operator inputs
        if( backwardDriveControl ) {
            yTranslation = -yTranslation;
            xTranslation = -xTranslation;
          //rotation     = -rotation;  // clockwise/counterclockwise doesn't change
        } // backwardDriveControl
        // Normal teleop drive control:
        // - left joystick is TRANSLATE fwd/back/left/right
        // - right joystick is ROTATE clockwise/counterclockwise
        // NOTE: assumes the right motors are defined FORWARD and the
        // left motors are defined REVERSE so positive power is FORWARD.
        frontRight = yTranslation - xTranslation + rotation;
        frontLeft  = yTranslation + xTranslation - rotation;
        rearRight  = yTranslation + xTranslation + rotation;
        rearLeft   = yTranslation - xTranslation - rotation;
        // Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(rearLeft),  Math.abs(rearRight)  ),
                             Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if (maxPower > 1.0)
        {
            rearLeft   /= maxPower;
            rearRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }
        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );
    } // processStandardDriveMode

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Driver-centric Mecanum-wheel drive control (depends on gyro!)         */
    /*---------------------------------------------------------------------------------*/
    void processDriverCentricDriveMode() {
        double leftFrontAngle, rightFrontAngle, leftRearAngle, rightRearAngle;
        double gyroAngle;

        // Retrieve X/Y and ROTATION joystick input
        if( controlMultSegLinear ) {
            yTranslation = multSegLinearXY( -gamepad1.left_stick_y );
            xTranslation = multSegLinearXY(  gamepad1.left_stick_x );
            rotation     = multSegLinearRot( -gamepad1.right_stick_x );
        }
        else {
            yTranslation = -gamepad1.left_stick_y;
            xTranslation = gamepad1.left_stick_x;
            rotation = -gamepad1.right_stick_x;
        }
        gyroAngle = -robot.headingIMU();

        if (gamepad1.square) {
            // The driver presses SQUARE, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as SQUARE is pressed, and will
            // not drive the robot using the left stick.  Once SQUARE is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
            // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis
            driverAngle = -Math.toDegrees( Math.atan2( -gamepad1.left_stick_x, gamepad1.left_stick_y) );
            if (driverAngle < 0) {
                driverAngle += 360.0;
            }
            driverAngle -= gyroAngle;
            xTranslation = 0.0;
            yTranslation = 0.0;
            rotation     = 0.0;
        }

        // Adjust new gyro angle for the driver reference angle
        gyroAngle += driverAngle;

        // Compute motor angles relative to current orientation
        rightFrontAngle = Math.toRadians( gyroAngle + 315 );  //   /    pulls at 315deg (135+180)
        leftFrontAngle  = Math.toRadians( gyroAngle + 45  );  //   \    pulls at 45deg
        rightRearAngle  = Math.toRadians( gyroAngle + 225 );  //   \    pulls at 225deg (45+180)
        leftRearAngle   = Math.toRadians( gyroAngle + 135 );  //   /    pulls at 135

        frontRight = (yTranslation * Math.sin(rightFrontAngle) + xTranslation * Math.cos(rightFrontAngle))/Math.sqrt(2) + rotation;
        frontLeft  = (yTranslation * Math.sin(leftFrontAngle)  + xTranslation * Math.cos(leftFrontAngle))/Math.sqrt(2)  + rotation;
        rearRight  = (yTranslation * Math.sin(rightRearAngle)  + xTranslation * Math.cos(rightRearAngle))/Math.sqrt(2)  + rotation;
        rearLeft   = (yTranslation * Math.sin(leftRearAngle)   + xTranslation * Math.cos(leftRearAngle))/Math.sqrt(2)   + rotation;

        // Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(rearLeft),  Math.abs(rearRight)  ),
                             Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if (maxPower > 1.0)
        {
            rearLeft   /= maxPower;
            rearRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }

        // Update motor power settings (left motors are defined as REVERSE mode)
        robot.driveTrainMotors( -frontLeft, frontRight, -rearLeft, rearRight );

    } // processDriverCentricDriveMode


    /**
     * Ensure angle is in the range of -PI to +PI (-180 to +180 deg)
     * @param angleRadians
     * @return
     */
    public double AngleWrapRadians( double angleRadians ){
        while( angleRadians < -Math.PI ) {
            angleRadians += 2.0*Math.PI;
        }
        while( angleRadians > Math.PI ){
            angleRadians -= 2.0*Math.PI;
        }
        return angleRadians;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        int leftChange  = robot.leftOdometerCount  - robot.leftOdometerPrev;
        int rightChange = robot.rightOdometerCount - robot.rightOdometerPrev;
        //Calculate Angle
        double changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians += changeInRobotOrientation;
        robotOrientationRadians = AngleWrapRadians( robotOrientationRadians );   // Keep between -PI and +PI
        //Get the components of the motion
        int rawHorizontalChange = robot.strafeOdometerCount - robot.strafeOdometerPrev;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);
        double p = ((rightChange + leftChange) / 2.0);
        double n = horizontalChange;
        //Calculate and update the position values
        robotGlobalXCoordinatePosition += (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition += (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));
    } // globalCoordinatePositionUpdate

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagID(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // If you do not manually specify calibration parameters, the SDK will
                // to load a predefined calibration for your camera.
                //  ===== CAMERA CALIBRATION for 150deg webcam ===
                //.setLensIntrinsics(332.309,332.309,341.008,243.109)
                //  ===== CAMERA CALIBRATION for Arducam B0197 webcam ===
                //.setLensIntrinsics(1566.16,1566.16,1002.58,539.862)
                //  ===== CAMERA CALIBRATION for Arducam B0385 webcam ===
                .setLensIntrinsics(904.214,904.214,696.3,362.796)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam Back"))
                .setCameraResolution(new Size(1280,800))
                .addProcessor(aprilTag)
                .build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

} // Teleop
