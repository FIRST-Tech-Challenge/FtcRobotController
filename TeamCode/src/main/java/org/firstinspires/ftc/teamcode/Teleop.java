/* FTC Team 7572 - Version 1.0 (10/28/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp Full Control.
 */
//@Disabled
public abstract class Teleop extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;  // 
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;  // 
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;  // 
    boolean gamepad1_square_last,     gamepad1_square_now     = false;  // 
//  boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;  // gamepad1.dpad_up used live/realtime
//  boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;  //   (see processDpadDriveMode() below)
//  boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
//  boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
//  boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;
//  boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;
    boolean gamepad1_touchpad_last,   gamepad1_touchpad_now   = false;  // autodrive to cone storage area

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  // Raise lift to TRANSPORT position
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  // Flip intake (toggle)
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  // Lower lift to COLLECT position
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  // UNUSED
    boolean gamepad2_dpad_up_last,    gamepad2_dpad_up_now    = false;  // Lift to HIGH junction
    boolean gamepad2_dpad_down_last,  gamepad2_dpad_down_now  = false;  // Lift to MEDIUM junction
    boolean gamepad2_dpad_left_last,  gamepad2_dpad_left_now  = false;  // Lift to LOW junction
    boolean gamepad2_dpad_right_last, gamepad2_dpad_right_now = false;  // Lower to GROUND junction
    boolean gamepad2_l_bumper_last,   gamepad2_l_bumper_now   = false;  // Collect cone (intake cone)
    boolean gamepad2_r_bumper_last,   gamepad2_r_bumper_now   = false;  // Deposit cone (eject cone)
    boolean gamepad2_touchpad_last,   gamepad2_touchpad_now   = false;  // UNUSED
    boolean gamepad2_share_last,      gamepad2_share_now      = false;  // UNUSED

    double  yTranslation, xTranslation, rotation;                  /* Driver control inputs */
    double  rearLeft, rearRight, frontLeft, frontRight, maxPower;  /* Motor power levels */
    boolean backwardDriveControl = false; // drive controls backward (other end of robot becomes "FRONT")
    boolean controlMultSegLinear = true;

    final int DRIVER_MODE_STANDARD     = 2;
    final int DRIVER_MODE_DRV_CENTRIC  = 3;
    int       driverMode               = DRIVER_MODE_STANDARD;
//  int       driverMode               = DRIVER_MODE_DRV_CENTRIC;
    double    driverAngle              = 0.0;  /* for DRIVER_MODE_DRV_CENTRIC */
    boolean   autoDrive                = false;

    // These are set in the alliance-specific teleops
    double      duckVelocityNow;
    double      duckVelocityStep;

    double    sonarRangeL=0.0, sonarRangeR=0.0, sonarRangeF=0.0, sonarRangeB=0.0;
    boolean   rangeSensorsEnabled = true;  // enable only when designing an Autonomous plan (takes time!)
    int       rangeSensorIndex = 1;         // only send a new ping out every other control cycle, and rotate sensors
    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    elapsedTime, elapsedHz;

    boolean   turretTweaked = false;     // Reminder to zero power when input stops
    boolean   liftTweaked = false;       // Reminder to zero power when input stops
    boolean   collectorFlipped = false;  // Collector has been flipped to upside down orientation

    ElapsedTime grabberRunTimer = new ElapsedTime();
    boolean     grabberIntake   = true;
    boolean     grabberRunning  = false;

    /* Declare OpMode members. */
    HardwareSlimbot robot = new HardwareSlimbot();

    // sets unique behavior based on alliance
    public abstract void setAllianceSpecificBehavior();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();
/*
        coneRumbleEffect1 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 500 mSec
                .build();
        coneRumbleEffect2 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 500 mSec
                .build();
*/
        // Initialize robot hardware
        robot.init(hardwareMap,false);

        setAllianceSpecificBehavior();

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

            // If enabled, process ultrasonic range sensors
            if( rangeSensorsEnabled ) {
                // measure the next sensor
                switch( rangeSensorIndex ) {
                    case 1 : processRangeSensors(rangeSensorIndex); break;
                    case 2 : break;  // nothing (skip this control cycle)
                    case 3 : processRangeSensors(rangeSensorIndex); break;
                    case 4 : break;  // nothing (skip this control cycle)
                    case 5 : processRangeSensors(rangeSensorIndex); break;
                    case 6 : break;  // nothing (skip this control cycle)
                    case 7 : processRangeSensors(rangeSensorIndex); break;
                    case 8 : break;  // nothing (skip this control cycle)
                }
                // increment to next index
                if( ++rangeSensorIndex > 8 )
                    rangeSensorIndex = 1;
            } // rangeSensorsEnabled

            // Process all the driver/operator inputs
            processTurretControls();
            processLiftControls();
            processGrabberControls();

            // Execute any automatic movements
            robot.liftPosRun();

/* DISABLE DRIVER-CENTRIC MODE FOR THIS SEASON
            // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button (toggles DRIVER-CENTRIC drive control)
            if( gamepad1_square_now && !gamepad1_square_last)
            {
                driverMode = DRIVER_MODE_DRV_CENTRIC;
            }
*/

/* DISABLE BACKWARD MODE FOR THIS SEASON
            // Check for an OFF-to-ON toggle of the gamepad1 TRIANGLE button (toggles STANDARD/BACKWARD drive control)
            if( gamepad1_triangle_now && !gamepad1_triangle_last)
            {
                // If currently in DRIVER-CENTRIC mode, switch to STANDARD (robot-centric) mode
                if( driverMode != DRIVER_MODE_STANDARD ) {
                    driverMode = DRIVER_MODE_STANDARD;
                    backwardDriveControl = false;  // reset to forward mode
                }
                // Already in STANDARD mode; Just toggle forward/backward mode
                else {  //(disabled for now)
                    backwardDriveControl = !backwardDriveControl; // reverses which end of robot is "FRONT"
                }
            }
*/
            // See if it's time to stop auto driving
//          processAutoDriveMode();

            if( processDpadDriveMode() == false ) {
                // Control based on joystick; report the sensed values
                telemetry.addData("Joystick", "x=%.3f, y=%.3f spin=%.3f",
                        -gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x );
                switch( driverMode ) {
                    case DRIVER_MODE_STANDARD :
                        telemetry.addData("Driver Mode", "STD%s (cir)", (backwardDriveControl)? "-BACKWARD":"" );
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
            telemetry.addData("Front", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
                    frontLeft, robot.frontLeftMotorVel, frontRight, robot.frontRightMotorVel );
            telemetry.addData("Rear ", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
                    rearLeft,  robot.rearLeftMotorVel,  rearRight,  robot.rearRightMotorVel );
            telemetry.addData("Odometry (L/R)", "%d %d cts",  robot.leftOdometerCount, robot.rightOdometerCount );
            telemetry.addData("Turret", "%d cts  %.1f deg  %.2f mA",
                    robot.turretMotorPos, robot.turretAngle, robot.turretMotorAmps );
            telemetry.addData("Lift",   "%.1f deg  %.2f pwr  %.2f mA",
                    robot.liftAngle, robot.liftMotorPwr, robot.liftMotorAmps );
            if( rangeSensorsEnabled ) {
               telemetry.addData("Sonar Range (L/R)", "%.1f  %.1f in", sonarRangeL/2.54, sonarRangeR/2.54 );
               telemetry.addData("Sonar Range (F/B)", "%.1f  %.1f in", sonarRangeF/2.54, sonarRangeB/2.54 );
            }
            telemetry.addData("Gyro Angle", "%.1f deg (%.1f tilt)", robot.headingIMU(), robot.tiltAngle );
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", elapsedTime, elapsedHz );
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
//      gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
//      gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
//      gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
//      gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
//      gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
//      gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
//      gamepad1_touchpad_last   = gamepad1_touchpad_now;    gamepad1_touchpad_now   = gamepad1.touchpad;
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
    /*  TELE-OP: Mecanum-wheel drive control using Dpad (slow/fine-adjustment mode)    */
    /*---------------------------------------------------------------------------------*/
    boolean processDpadDriveMode() {
        double fineDriveSpeed  = 0.21;
        double fineStrafeSpeed = 0.35;
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
        else if( autoDrive || (gamepad1_touchpad_now && !gamepad1_touchpad_last) ) {
            telemetry.addData("Touchpad","FORWARD");
            frontLeft  = autoDriveSpeed;
            frontRight = autoDriveSpeed;
            rearLeft   = autoDriveSpeed;
            rearRight  = autoDriveSpeed;
            autoDrive = true;
        }
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

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Capture range-sensor data (one reading! call from main control loop)  */
    /*                                                                                 */
    /*  Designed for test programs that are used to assess the mounting location of    */
    /*  your sensors and whether you get reliable/repeatable returns off various field */
    /*  elements.                                                                      */
    /*                                                                                 */
    /*  IMPORTANT!! updateSonarRangeL / updateSonarRangeR may call getDistanceSync(),  */
    /*  which sends out an ultrasonic pulse and SLEEPS for the sonar propogation delay */
    /*  (50 sec) before reading the range result.  Don't use in applications where an  */
    /*  extra 50/100 msec (ie, 1 or 2 sensors) in the loop time will create problems.  */
    /*  If getDistanceAsync() is used, then this warning doesn't apply.                */
    /*---------------------------------------------------------------------------------*/
    void processRangeSensors( int sensorNum ) {
        // only send one ping per control cycle (left, right, front, or back)
        switch( sensorNum ) {
            case 1 : sonarRangeL = robot.updateSonarRangeL(); break;
            case 3 : /* sonarRangeR = robot.updateSonarRangeR(); */ break;
            case 5 : sonarRangeF = robot.updateSonarRangeF(); break;
            case 7 : sonarRangeB = robot.updateSonarRangeB(); break;
            default : break;
        } // switch()
    } // processRangeSensors

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: averaged range-sensor data (multiple readings!)                       */
    /*                                                                                 */
    /*  Designed for applications where continuous range updates are unnecessary, but  */
    /*  we want to know the correct distance "right now".                              */
    /*---------------------------------------------------------------------------------*/
    void averagedRangeSensors() {
        // repeatedly update all 4 readings.  Each loop adds a reading to the
        // internal array from which we return the new MEDIAN value.
        for( int i=0; i<5; i++ ) {
          sonarRangeL = robot.updateSonarRangeL();
//        sonarRangeR = robot.updateSonarRangeR();
          sonarRangeF = robot.updateSonarRangeF();
          sonarRangeB = robot.updateSonarRangeB();
        }
    } // averagedRangeSensors

    /*---------------------------------------------------------------------------------*/
    void processTurretControls() {
        boolean safeToManuallyLeft  = (robot.turretMotorPos > robot.TURRET_LIMIT_LEFT);
        boolean safeToManuallyRight = (robot.turretMotorPos < robot.TURRET_LIMIT_RIGHT );
        double  gamepad2_left_stick = gamepad2.left_stick_x;
        boolean manual_turret_control = ( Math.abs(gamepad2_left_stick) > 0.05 );

        if( manual_turret_control || turretTweaked ) {
            // Does user want to rotate turret LEFT (negative joystick input)
            if( safeToManuallyLeft && (gamepad2_left_stick < -0.05) ) {
                double motorPower = multSegLinearRot( gamepad2_left_stick ); // NEGATIVE
                robot.turretMotor.setPower( motorPower );   // 0% to -100%
                turretTweaked = true;
            }
            // Does user want to rotate turret RIGHT (positive joystick input)
            else if( safeToManuallyRight && (gamepad2_left_stick > 0.05) ) {
                double motorPower = multSegLinearRot( gamepad2_left_stick ); // POSITIVE
                robot.turretMotor.setPower( motorPower );   // 0% to +100%
                turretTweaked = true;
            }
            // No more input?  Time to stop turret movement!
            else if( turretTweaked ) {
                robot.turretMotor.setPower( 0.0 );
                turretTweaked = false;
            }
        } // manual_turret_control

    } // processTurretControls

    /*---------------------------------------------------------------------------------*/
    // When the lift is at floor level (+110 degrees) it requires 0.30 minimum
    // power just to begin lifting
    double computeLiftMotorPower( double joystick ) {
        double minPower, motorPower;
        // Are we working on the front side of the robot?
        if( robot.liftAngle > 0 ) {
            // Are we lowering (-) or raising (+) ?
            minPower = (joystick < 0)? -0.10 : (0.30 * (robot.liftAngle / robot.LIFT_ANGLE_MAX));
        }
        else {
            // TODO: finish this once we establish operation on the back side of the bot
            minPower = (joystick < 0)? -0.10 : 0.10;
        }
        motorPower = minPower + joystick;
        if( motorPower >  1.0 ) motorPower =  1.0;
        if( motorPower < -1.0 ) motorPower = -1.0;
        return motorPower;
    } // computeLiftMotorPower

    /*---------------------------------------------------------------------------------*/
    void processLiftControls() {
        boolean safeToManuallyLower = (robot.liftAngle < robot.LIFT_ANGLE_MAX);
        boolean safeToManuallyRaise = (robot.liftAngle > robot.LIFT_ANGLE_MIN);
        double  gamepad2_right_stick = -gamepad2.right_stick_y;
        boolean manual_lift_control = ( Math.abs(gamepad2_right_stick) > 0.05 );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 CROSS button
        if( gamepad2_cross_now && !gamepad2_cross_last)
        {   // Lower lift to COLLECT position and adjust collector tilt horizontal
            robot.grabberSpinStop();
            robot.grabberSetTilt( robot.GRABBER_TILT_GRAB );
            robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
        }
        // Check for an OFF-to-ON toggle of the gamepad2 CIRCLE button
        else if( gamepad2_circle_now && !gamepad2_circle_last )
        {   // Flip intake (toggle)
            if( collectorFlipped ) {
                robot.rotateServo.setPosition( robot.GRABBER_ROTATE_UP );
                collectorFlipped = false;
            }
            else {
                robot.rotateServo.setPosition( robot.GRABBER_ROTATE_DOWN );
                collectorFlipped = true;
            }
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 LEFT BUMPER
        else if( gamepad2_l_bumper_now && !gamepad2_l_bumper_last )
        {   // intake cone
            grabberRunTimer.reset();
            robot.grabberSpinCollect();
            grabberRunning  = true;
            grabberIntake   = true;
            // start slowly lowering onto cone
            robot.liftMotorsSetPower( -0.15 );
        }
        // Check for an OFF-to-ON toggle of the gamepad2 RIGHT BUMPER
        else if( gamepad2_r_bumper_now && !gamepad2_r_bumper_last )
        {   // eject cone
            grabberRunTimer.reset();
            robot.grabberSpinEject();
            grabberRunning  = true;
            grabberIntake   = false;
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD UP
        else if( gamepad2_dpad_up_now && !gamepad2_dpad_up_last)
        {   // Raise lift to HIGH junction
            robot.grabberSpinStop();
            robot.grabberSetTilt( robot.GRABBER_TILT_STORE );
            robot.liftPosInit( robot.LIFT_ANGLE_HIGH );
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD LEFT
        else if( gamepad2_dpad_left_now && !gamepad2_dpad_left_last)
        {   // Raise lift to MEDIUM junction
            robot.grabberSpinStop();
            robot.grabberSetTilt( robot.GRABBER_TILT_STORE );
            robot.liftPosInit( robot.LIFT_ANGLE_MED );
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD RIGHT
        else if( gamepad2_dpad_right_now && !gamepad2_dpad_right_last)
        {   // Raise lift to LOW junction
            robot.grabberSpinStop();
            robot.grabberSetTilt( robot.GRABBER_TILT_STORE );
            robot.liftPosInit( robot.LIFT_ANGLE_LOW );
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD DOWN
        else if( gamepad2_dpad_down_now && !gamepad2_dpad_down_last)
        {   // Lower to GROUND junction
            robot.grabberSpinStop();
            robot.grabberSetTilt( robot.GRABBER_TILT_STORE );
            robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
        }
        else if( manual_lift_control || liftTweaked ) {
            // Does user want to rotate lift toward more NEGATIVE counts (negative joystick input)
            if( safeToManuallyRaise && (gamepad2_right_stick > 0.05) ) {
                double motorPower = computeLiftMotorPower( gamepad2_right_stick ); // POSITIVE
                robot.liftMotorsSetPower( motorPower );
                liftTweaked = true;
            }
            // Does user want to rotate lift toward more POSITIVE counts (positive joystick input)
            else if( safeToManuallyLower && (gamepad2_right_stick < -0.05) ) {
                double motorPower = computeLiftMotorPower( gamepad2_right_stick ); // NEGATIVE
                robot.liftMotorsSetPower( motorPower );
                liftTweaked = true;
            }
            // No more input?  Time to stop lift movement!
            else if( liftTweaked ) {
                robot.liftMotorsSetPower( 0.0 );
                liftTweaked = false;
            }
        } // manual_lift_control

    } // processLiftControls

    /*---------------------------------------------------------------------------------*/
    void processGrabberControls() {
        // Anything to process?
        if( grabberRunning ) {
            // Current on an INTAKE cycle?
            if (grabberIntake) {
                // Is cycle complete?
                if( grabberRunTimer.milliseconds() >= 500 ) {
                    // stop lowering
                    robot.liftMotorsSetPower( 0.0 );
                    // stop collecting
                    robot.grabberSpinStop();
                    grabberRunning = false;
                }
            } // intake
            // Currently on an EJECTION cycle?
            else {
                // Is cycle complete?
                if( grabberRunTimer.milliseconds() >= 750 ) {
                    // stop ejecting cone
                    robot.grabberSpinStop();
                    grabberRunning = false;
                }
            } // ejection
        } // grabberRunning
    } // processGrabberControls

} // Teleop
