/* FTC Team 7572 - Version 1.0 (11/10/2023)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

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

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  // Lower lift to collect from current stack height
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  // Flip intake (toggle)
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  // Lower lift to COLLECT position
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  // Raise lift to TRANSPORT position
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

    Gamepad.RumbleEffect coneRumbleEffect1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect coneRumbleEffect2;    // Use to build a custom rumble sequence.

    // sets unique behavior based on alliance
    public abstract void setAllianceSpecificBehavior();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        coneRumbleEffect1 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble left/right motors 100% for 500 mSec
                .build();
        coneRumbleEffect2 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 500 mSec
                .build();

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
            globalCoordinatePositionUpdate();

            ProcessCollectorControls();

            // Check for an OFF-to-ON toggle of the gamepad1 TRIANGLE button
            if( gamepad1_triangle_now && !gamepad1_triangle_last)
            {

            }

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

            telemetry.addData("circle","Robot-centric (fwd/back modes)");
            telemetry.addData("square","Driver-centric (set joystick!)");
            telemetry.addData("d-pad","Fine control (30%)");
            telemetry.addData(" "," ");

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

    void ProcessCollectorControls() {
        // Check for an OFF-to-ON toggle of the gamepad2 CROSS button (turns off collector motor)
        if( gamepad2_cross_now && !gamepad2_cross_last)
        {
            robot.collectorMotor.setPower(0.0);
            robot.collectorServo.setPosition(robot.COLLECTOR_SERVO_RAISED);
        }

        if( gamepad2_l_bumper_now && !gamepad2_l_bumper_last)
        {
            robot.collectorMotor.setPower(-robot.COLLECTOR_MOTOR_POWER);
        }

        if( gamepad2_r_bumper_now && !gamepad2_r_bumper_last)
        {
            robot.collectorMotor.setPower(robot.COLLECTOR_MOTOR_POWER);
        }

        // Check for an OFF-to-ON toggle of the gamepad2 CIRCLE button
        if( gamepad2_circle_now && !gamepad2_circle_last)
        {
            robot.collectorServo.setPosition(robot.COLLECTOR_SERVO_GROUND);
        }

        // Check for an OFF-to-ON toggle of the gamepad2 TRIANGLE button
        if( gamepad2_triangle_now && !gamepad2_triangle_last)
        {
            robot.collectorServo.setPosition(robot.COLLECTOR_SERVO_RAISED);
        }

    }  // processCollectorControls

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

} // Teleop
