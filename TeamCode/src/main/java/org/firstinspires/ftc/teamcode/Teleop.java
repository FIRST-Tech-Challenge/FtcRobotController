/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

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
//  int       driverMode               = DRIVER_MODE_STANDARD;
    int       driverMode               = DRIVER_MODE_DRV_CENTRIC;
    double    driverAngle              = 180.0;  /* for DRIVER_MODE_DRV_CENTRIC */

    boolean   batteryVoltsEnabled = false;  // enable only during testing (takes time!)
    boolean   robotCentricDriveEnabled = false;  // DON'T LET STUDENTS USE THIS!

    double    viperPower = 0.0;
    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    cycleTimeElapsed, cycleTimeHz;

    /* Declare OpMode members. */
    Hardware2025Bot robot = new Hardware2025Bot();

    boolean leftAlliance = true;  // overriden in setAllianceSpecificBehavior()
    boolean blueAlliance = true;  // overriden in setAllianceSpecificBehavior()

    int     aprilTagHuman  = 11;  // overriden in setAllianceSpecificBehavior() Default to Blue Alliance
    int     aprilTagStart  = 12;  // overriden in setAllianceSpecificBehavior() Default to Blue Alliance
    int     aprilTagBasket = 13;  // overriden in setAllianceSpecificBehavior() Default to Blue Alliance

    boolean snorkleTweaked       = false; // Reminder to zero power when PAN  input stops
    boolean tiltAngleTweaked     = false; // Reminder to zero power when TILT input stops
    boolean liftTweaked          = false; // Reminder to zero power when LIFT input stops
    boolean enableOdometry       = true; // Process/report odometry updates?
    int     grabLState               = 2;
    int     grabRState               = 2;
    int     submersibleCollectState  = 2;

    double    clawServoPosAdj = 0.500;

    double   curX, curY, curAngle;
    double   minX=0.0, maxX=0.0, minY=0.0, maxY=0.0;

    final int ASCENT_STATE_IDLE   = 0;
    final int ASCENT_STATE_SETUP  = 1;
    final int ASCENT_STATE_SETUP_MOVING = 2;
    final int ASCENT_STATE_SETUP_READY = 3;
    final int ASCENT_STATE_SNOKEL_LIFTING = 4;
    final int ASCENT_STATE_ARM_PREP = 5;
    final int ASCENT_STATE_HOOKING  = 6;
    final int ASCENT_STATE_LIFTING  = 7;
    final int ASCENT_STATE_HANGING  = 8;

    int         ascent2state = ASCENT_STATE_IDLE;
	boolean     ascent2telem = false;

    protected ElapsedTime ascent2Timer = new ElapsedTime();

    Gamepad.RumbleEffect visibleAprilTagRumble1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect rumblePixelBinSingle;
    Gamepad.RumbleEffect rumbleAscentReady;

    // sets unique behavior based on alliance
    public abstract void setAllianceSpecificBehavior();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        visibleAprilTagRumble1 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble left/right motors 100% for 500 mSec
                .build();
        rumblePixelBinSingle = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 250)  //  Rumble LEFT motor 100% for 250 mSec
                .build();   // use different sides for SINGLE and DOUBLE in case they're signalled back-to-back

        rumbleAscentReady = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 250)  //  Rumble RIGHT motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(0.0, 1.0, 250)  //  Rumble RIGHT motor 100% for 250 mSec
                .build();

        // Initialize robot hardware (not autonomous mode)
        robot.init(hardwareMap,false);

        setAllianceSpecificBehavior();

        // Get the default start angle (set during Autonomous)
        driverAngle = robot.rcStartAngleGet();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "Ready");
        telemetry.addLine("Press X (cross) to reset encoders");
        telemetry.addLine("(to run Teleop without Auto first)");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Normally autonomous resets encoders.  Do we need to for teleop??
            if( gamepad1_cross_now && !gamepad1_cross_last) {
                robot.resetEncoders();
            }
            // Pause briefly before looping
            idle();
        } // !isStarted

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Refresh gamepad button status
            captureGamepad1Buttons();
            captureGamepad2Buttons();

            // Bulk-refresh the Control/Expansion Hub device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();

            // Request an update from the Pinpoint odometry computer (single I2C read)
            if( enableOdometry ) {
                robot.odom.update();
                Pose2D pos = robot.odom.getPosition();  // x,y pos in inch; heading in degrees
                curX     = pos.getX(DistanceUnit.INCH);
                curY     = pos.getY(DistanceUnit.INCH);
                curAngle = pos.getHeading(AngleUnit.DEGREES);
                String posStr = String.format(Locale.US, "{X,Y: %.1f, %.1f in  H: %.1f deg}", curX, curY, curAngle);
                telemetry.addData("Position", posStr);
                //==== TEMPORARY ODOMETRY CALIBRATION CODE ============================================================
//              if(curX<minX){minX=curX;} if(curX>maxX){maxX=curX;}
//              if(curY<minY){minY=curY;} if(curY>maxY){maxY=curY;}
//              double x_radius_mm = 25.4 * (maxX-minX)/2.0;  // rotate 180deg; max-min is the diameter of the circle
//              double y_radius_mm = 25.4 * (maxY-minY)/2.0;  // of error relative to the true center of the robot
//              telemetry.addData("Odo Circle", "x=%.2f, y=%.2f mm", x_radius_mm, y_radius_mm );
                //=====================================================================================================
                Pose2D vel = robot.odom.getVelocity(); // x,y velocities in inch/sec; heading in deg/sec
                String velStr = String.format(Locale.US,"{X,Y: %.1f, %.1f in/sec, HVel: %.2f deg/sec}",
                     vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Velocity", velStr);
                telemetry.addData("Status", robot.odom.getDeviceStatus());
            }

            // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button (toggles DRIVER-CENTRIC drive control)
            if( gamepad1_square_now && !gamepad1_square_last)
            {
                driverMode = DRIVER_MODE_DRV_CENTRIC;
            }

            // Check for an OFF-to-ON toggle of the gamepad1 CIRCLE button (toggles STANDARD/BACKWARD drive control)
            if( robotCentricDriveEnabled ) {
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
            } // robotCentricDriveEnabled

//          telemetry.addData("circle","Robot-centric (fwd/back modes)");
//          telemetry.addData("square","Driver-centric (set joystick!)");
//          telemetry.addData("d-pad","Fine control (30%)");
//          telemetry.addData(" "," ");

            if( processDpadDriveMode() == false ) {
                // Control based on joystick; report the sensed values
//              telemetry.addData("Joystick1", "x=%.3f, y=%.3f spin=%.3f",
//                      gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x );
//              telemetry.addData("Joystick2", "pan=%.3f, tilt=%.3f extend=%.3f",
//                      gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_y );
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

            processTiltControls();
            ProcessViperLiftControls();
            processClaw();
            processSnorkleControls();
            performEveryLoopTeleop();
            processLevel2Ascent();

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            cycleTimeElapsed = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            cycleTimeHz =  1000.0 / cycleTimeElapsed;

            // Update telemetry data
//          telemetry.addData("Front", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
//                  frontLeft, robot.frontLeftMotorVel, frontRight, robot.frontRightMotorVel );
//          telemetry.addData("Rear ", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
//                  rearLeft,  robot.rearLeftMotorVel,  rearRight,  robot.rearRightMotorVel );
//          telemetry.addData("Front", "%d %d counts", robot.frontLeftMotorPos, robot.frontRightMotorPos );
//          telemetry.addData("Back ", "%d %d counts", robot.rearLeftMotorPos,  robot.rearRightMotorPos );
            telemetry.addData("Snorkel", "L = %d, R = %d counts", robot.snorkleLMotorPos, robot.snorkleRMotorPos );
            telemetry.addData("Tilt", "%d counts %.1f deg", robot.wormTiltMotorPos, robot.armTiltAngle );
            telemetry.addData("Viper", "%d counts ", robot.viperMotorPos );
            telemetry.addData("Elbow", "%.2f (%.1f deg)", robot.getElbowServoPos(), robot.getElbowServoAngle() );
            telemetry.addData("Wrist", "%.2f (%.1f deg)", robot.getWristServoPos(), robot.getElbowServoAngle() );
            telemetry.addData("Snorkle Ascent State", ascent2state);
//          telemetry.addData("Angles", "IMU %.2f, Pinpoint %.2f deg)", robot.headingIMU(), curAngle );
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", cycleTimeElapsed, cycleTimeHz);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//          robot.waitForTick(40);
        } // opModeIsActive

    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void performEveryLoopTeleop() {
        robot.processViperSlideExtension();
        robot.processWormTilt();
        processHoverArm();
        processSecureArm();
        processScoreArm();
        processScoreArmSpec();
        //processSweeper();
    } // performEveryLoopTeleop

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
        gamepad1_touchpad_last   = gamepad1_touchpad_now;    gamepad1_touchpad_now   = gamepad1.touchpad;
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
        gamepad2_touchpad_last   = gamepad2_touchpad_now;    gamepad2_touchpad_now   = gamepad2.touchpad;
//      gamepad2_share_last      = gamepad2_share_now;       gamepad2_share_now      = gamepad2.share;
    } // captureGamepad2Buttons

    /*  TELE-OP: Mecanum-wheel drive control using Dpad (slow/fine-adjustment mode)    */
    /*---------------------------------------------------------------------------------*/
    boolean processDpadDriveMode() {
        double fineControlSpeed = 0.15;
        boolean dPadMode = true;
        // Only process 1 Dpad button at a time
        if( gamepad1.dpad_up ) {
            telemetry.addData("Dpad","FORWARD");
            frontLeft  = fineControlSpeed;
            frontRight = fineControlSpeed;
            rearLeft   = fineControlSpeed;
            rearRight  = fineControlSpeed;
        }
        else if( gamepad1.dpad_down ) {
            telemetry.addData("Dpad","BACKWARD");
            frontLeft  = -fineControlSpeed;
            frontRight = -fineControlSpeed;
            rearLeft   = -fineControlSpeed;
            rearRight  = -fineControlSpeed;
        }
        else if( gamepad1.dpad_left ) {
            telemetry.addData("Dpad","LEFT");
            frontLeft  = -fineControlSpeed;
            frontRight =  fineControlSpeed;
            rearLeft   =  fineControlSpeed;
            rearRight  = -fineControlSpeed;
        }
        else if( gamepad1.dpad_right ) {
            telemetry.addData("Dpad","RIGHT");
            frontLeft  =  fineControlSpeed;
            frontRight = -fineControlSpeed;
            rearLeft   = -fineControlSpeed;
            rearRight  =  fineControlSpeed;
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
        if( controlMultSegLinear ) {  // robot centric results in 1.0 max power
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
        if( controlMultSegLinear ) { // // driver centric results in 0.6 max??
            yTranslation = 1.66 * multSegLinearXY( -gamepad1.left_stick_y );
            xTranslation = 1.66 * multSegLinearXY(  gamepad1.left_stick_x );
            rotation     = 1.66 * multSegLinearRot( -gamepad1.right_stick_x );
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
    void processSnorkleControls() {
        boolean safeToManuallyLower  = (robot.snorkleLMotorPos > robot.SNORKLE_HW_MIN);
        boolean safeToManuallyRaise = (robot.snorkleLMotorPos < robot.SNORKLE_HW_MAX);
        double  gamepad2_left_stick = -gamepad2.left_stick_y;
        boolean manual_snorkle_control = ( Math.abs(gamepad2_left_stick) > 0.15 );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 LEFT BUMPER
        if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last )
        {
            // used elsewhere!
        }
        // Check for an OFF-to-ON toggle of the gamepad1 RIGHT BUMPER
        else if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last )
        {
        }

        //===================================================================
        else if( manual_snorkle_control || snorkleTweaked) {
            // Does user want to lower snorkle arm (negative joystick input)
            if( safeToManuallyLower && (gamepad2_left_stick < -0.15) ) {
                double motorPower = 1.00 * gamepad2_left_stick; // NEGATIVE
                robot.snorkleLMotor.setPower( motorPower );   // -3% to -20%
                robot.snorkleRMotor.setPower( motorPower );   // -3% to -20%
                snorkleTweaked = true;
            }
            // Does user want to raise snorkle RIGHT (positive joystick input)
            else if( safeToManuallyRaise && (gamepad2_left_stick > 0.15) ) {
                double motorPower = 1.00 * gamepad2_left_stick; // POSITIVE
                robot.snorkleLMotor.setPower( motorPower );   // +3% to +20%
                robot.snorkleRMotor.setPower( motorPower );   // +3% to +20%
                snorkleTweaked = true;
            }
            // No more input?  Time to stop turret movement!
            else if(snorkleTweaked) {
                robot.snorkleLMotor.setPower( 0.0 );
                robot.snorkleRMotor.setPower( 0.0 );
                snorkleTweaked = false;
            }
        } // manual_snorkle_control

    } // processSnorkleControls

    /*---------------------------------------------------------------------------------*/
    void processTiltControls() {
        // The encoder is backwards from our definition of MAX and MIN. Maybe change the
        // convention in hardware class?
        boolean safeToManuallyLower = (robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_HW_MIN_DEG);
        boolean safeToManuallyRaise = (robot.armTiltAngle < Hardware2025Bot.TILT_ANGLE_BASKET_DEG);
        double  gamepad2_right_stick = gamepad2.right_stick_y;
        boolean manual_tilt_control = ( Math.abs(gamepad2_right_stick) > 0.08 );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 CROSS button
        if( gamepad1_cross_now && !gamepad1_cross_last)
        {
            // robot.turretPIDPosInit( robot.PAN_ANGLE_CENTER );
        }

        //===================================================================
        else if( manual_tilt_control || tiltAngleTweaked) {
            terminateAutoArmMovements();
            robot.abortWormTilt();
            // Does user want to rotate turret DOWN (negative joystick input)
            if( safeToManuallyLower && (gamepad2_right_stick < -0.08) ) {
                double motorPower = 0.95 * gamepad2_right_stick; // NEGATIVE
                robot.wormTiltMotor.setPower( motorPower );   // -8% to -95%
                tiltAngleTweaked = true;
            }
            // Does user want to rotate turret UP (positive joystick input)
            else if( safeToManuallyRaise && (gamepad2_right_stick > 0.08) ) {
                double motorPower = 0.95 * gamepad2_right_stick; // POSITIVE
                robot.wormTiltMotor.setPower( motorPower );   // +8% to +95%
                tiltAngleTweaked = true;
            }
            // No more input?  Time to stop turret movement!
            else if(tiltAngleTweaked) {
                robot.wormTiltMotor.setPower( 0.0 );
                tiltAngleTweaked = false;
            }
        } // manual_tilt_control

    } // processTiltControls

    /*---------------------------------------------------------------------------------*/
    double ComputeMaxExtensionAtAngle(double tiltAngleDegrees) {
        double maxExtension;
        if(tiltAngleDegrees > Hardware2025Bot.TILT_ANGLE_42){
            maxExtension = Hardware2025Bot.VIPER_EXTEND_BASKET;
        } else {
            double tiltAngleRadians = Math.toRadians(tiltAngleDegrees);
            maxExtension = Hardware2025Bot.VIPER_EXTEND_42/Math.cos(tiltAngleRadians);
        }
        return maxExtension;
    }

    void ProcessViperLiftControls() {
        double  maxViperAtAngle       = ComputeMaxExtensionAtAngle(robot.armTiltAngle);
        boolean safeToManuallyRetract = (robot.viperMotorPos > Hardware2025Bot.VIPER_EXTEND_ZERO);
        boolean safeToManuallyExtend  = (robot.viperMotorPos < maxViperAtAngle);
        // Capture user inputs ONCE, in case they change during processing of this code
        // or we want to scale them down
        double  gamepad2_left_trigger  = gamepad2.left_trigger  * 1.00;
        double  gamepad2_right_trigger = gamepad2.right_trigger * 1.00;
        boolean manual_lift_control = ( (gamepad2_left_trigger  > 0.25) || (gamepad2_right_trigger > 0.25) );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD UP
        if( gamepad2_dpad_up_now && !gamepad2_dpad_up_last)
        {
            startScoreArm();
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD RIGHT
        else if( gamepad2_dpad_right_now && !gamepad2_dpad_right_last)
        {
            startScoreArmSpec();
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD DOWN
        else if( gamepad2_dpad_down_now && !gamepad2_dpad_down_last)
        {
            // Retract lift to the SAMPLE collection position
            startHoverArm();
            grabLState = 2;
            grabRState = 2;
            submersibleCollectState = 2; // forgot to update this state
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD LEFT
        else if( gamepad2_dpad_left_now && !gamepad2_dpad_left_last)
        {
            startSecureArm();
        }
        // Check for ON-to-OFF toggle of the gamepad2 SQUARE
        else if( gamepad2_square_now && !gamepad2_square_last )
        {   // position to grab specimen off field wall
            robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_WALL0_DEG);
            robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_WALL0);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_WALL0);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_WALL0);
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
        }
        // Check for ON-to-OFF toggle of the gamepad2 TRIANGLE
        else if( gamepad2_triangle_now && !gamepad2_triangle_last )
        {   // clip specimen on high bar REVERSE-SCORE!
            robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_AUTO4);
            sleep( 900 ); //while( autoViperMotorMoving() );
            // release the specimen
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
        }
        // Check for ON-to-OFF toggle of the gamepad2 CIRCLE
        else if( gamepad2_circle_now && !gamepad2_circle_last )
        {   // toggle between arm positions when in submersible
            if(submersibleCollectState == 2) {
                robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_TELEOP_COLLECT_DEG);
                submersibleCollectState = 1;
            } else {
                robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_SUBMERSIBLE_DEG);
                submersibleCollectState = 2;
            }
            //robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
        }
        // Check for ON-to-OFF toggle of the gamepad2 CROSS
        else if( gamepad2_cross_now && !gamepad2_cross_last )
        {
            //startSweeper();
        }
        //===================================================================
        else if( manual_lift_control || liftTweaked ) {
            // Does user want to manually EXTEND the lift?
            if( safeToManuallyExtend && (gamepad2_right_trigger > 0.25) ) {
                // Do we need to terminate an auto movement?
                double distanceTo42 = maxViperAtAngle - robot.viperMotorPos;
                terminateAutoArmMovements();
                robot.abortViperSlideExtension();
                viperPower = (distanceTo42 > 200)? gamepad2_right_trigger : 0.08;
                robot.viperMotor.setPower( viperPower );  // fixed power? (robot.VIPER_RAISE_POWER)
                liftTweaked = true;
            }
            // Does user want to manually RETRACT the lift?
            else if( safeToManuallyRetract && (gamepad2_left_trigger > 0.25) ) {
                // Do we need to terminate an auto movement?
                terminateAutoArmMovements();
                robot.abortViperSlideExtension();
                viperPower = robot.VIPER_LOWER_POWER;
                robot.viperMotor.setPower( viperPower );
                liftTweaked = true;
            }
            // No more input?  Time to stop lift movement!
            else if( liftTweaked ) {
                // if the lift is near the bottom, truly go to zero power
                // but if in a raised position, only drop to minimal holding power
                boolean viperCloseToZero = (Math.abs(robot.viperMotorPos - Hardware2025Bot.VIPER_EXTEND_ZERO) < 20);
                boolean tiltCloseToZero =  (robot.armTiltAngle < 45.0); // Hold power needed above this tilt angle
                viperPower = (viperCloseToZero || tiltCloseToZero)? 0.0 : robot.VIPER_HOLD_POWER;
                robot.viperMotor.setPower( viperPower );
                liftTweaked = false;
            }
        } // manual_lift_control

    }  // ProcessLiftControls

    /*---------------------------------------------------------------------------------*/
    void processClaw() {

        // Left Bumper = OPEN claw
        if( gamepad2_l_bumper_now && !gamepad2_l_bumper_last) {
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN );
        }

        // Right Bumper = CLOSE claw
        if( gamepad2_r_bumper_now && !gamepad2_r_bumper_last) {
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_CLOSED );
        }

        // Check for an OFF-to-ON toggle of the gamepad1 bumpers
        if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last) {
            if(grabLState == 2){
                robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRABL1);
                grabLState = 1;
            } else {
                robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRABL2);
                grabLState = 2;
            }
        }

        if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last) {
            if(grabRState == 2){
                robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRABR1);
                grabRState = 1;
            } else {
                robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRABR2);
                grabRState = 2;
            }
        }
    } // processClaw

    /*---------------------------------------------------------------------------------*/
    void processLevel2Ascent() {
        boolean motorsFinished, tiltArmFinished, snokelsFarEnough;

        // DRIVER 1 controls position the arm for hanging
        // DRIVER 2 controls initiate the actual hang

        // Check for emergency ASCENT ABORT button
        boolean player1abort = gamepad1_touchpad_now && !gamepad1_touchpad_last;
        boolean player2abort = gamepad2_touchpad_now && !gamepad2_touchpad_last;
        if( player1abort || player2abort ) {
            robot.viperMotor.setPower( 0.0 );
            robot.wormTiltMotor.setPower( 0.0 );
            robot.snorkleLMotor.setPower( 0.0 );
            robot.snorkleRMotor.setPower( 0.0 );
            robot.abortSnorkleExtension();
            ascent2state = ASCENT_STATE_IDLE;
			ascent2telem = false;
        }

        switch( ascent2state ) {
            case ASCENT_STATE_IDLE :
                // First instance of BOTH gamepad1 left/right bumpers initiates ascent prep
                if( gamepad1_l_bumper_now && gamepad1_r_bumper_now )
                {
                    terminateAutoArmMovements();
                    ascent2telem = true; // start monitoring motor powers
                    ascent2state = ASCENT_STATE_SETUP;
                }
                break;
            //===============================================================================================
            case ASCENT_STATE_SETUP:
                // Send Snorkel motors to raised position
                robot.startSnorkleExtension(Hardware2025Bot.SNORKLE_LEVEL2A, 1.0 );
                // Send TILT motor to hang position
                robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_LEVEL2A_DEG);
                // Send LIFT motor to hang position
                robot.startViperSlideExtension( Hardware2025Bot.VIPER_EXTEND_LEVEL2A);
                robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
                // bumpers may be changed the tilt the claw wrist/elbow... reset to desired angle
                robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_LEVEL2);
                robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
                ascent2state = ASCENT_STATE_SETUP_MOVING;
                break;
            case ASCENT_STATE_SETUP_MOVING:
                robot.processSnorkleExtension();
                motorsFinished = !robot.snorkleLMotorBusy && !robot.snorkleRMotorBusy &&
                                 !robot.viperMotorBusy && !robot.wormTiltMotorBusy;
                if( motorsFinished ) {
                    // Ready for phase 2
                    gamepad2.runRumbleEffect(rumbleAscentReady);
                    ascent2state = ASCENT_STATE_SETUP_READY;
                }
                break;
            //===============================================================================================
            case ASCENT_STATE_SETUP_READY:
                if( gamepad2_l_bumper_now && gamepad2_r_bumper_now ) {
                    // Fully retract snorkels
                    robot.startSnorkleExtension(Hardware2025Bot.SNORKLE_LEVEL2B, 1.0 );
                    robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_LEVEL2B_DEG);
                    robot.startViperSlideExtension( Hardware2025Bot.VIPER_EXTEND_LEVEL2B, 1.0, 1.0 );
                    robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );  // we accidentally open the claw
                    ascent2state = ASCENT_STATE_SNOKEL_LIFTING;
                    ascent2Timer.reset();
                }
                break;
            case ASCENT_STATE_SNOKEL_LIFTING:
                robot.processSnorkleExtension();
                tiltArmFinished = tiltAngleCloseEnough( Hardware2025Bot.TILT_ANGLE_LEVEL2B_DEG, 1.0 );
                // We can start the next motion when snorkels raise enough for robot to swing forward
                snokelsFarEnough = (robot.snorkleLMotorPos <= Hardware2025Bot.SNORKLE_LOW_BAR) &&
                                   (robot.snorkleRMotorPos <= Hardware2025Bot.SNORKLE_LOW_BAR);
                motorsFinished = tiltArmFinished && snokelsFarEnough && !robot.viperMotorBusy;
                if( motorsFinished || (ascent2Timer.milliseconds() > 2500.0) ) {
                    // Robot should now be hanging on snorkels, but butt is now resting on floor
                    // Reposition arm (tilt/extension) so grab upper bar to pull our butt off the floor
                    robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_LEVEL2C_DEG);
                    robot.startViperSlideExtension( Hardware2025Bot.VIPER_EXTEND_LEVEL2C, 1.0, 1.0 );
                    ascent2state = ASCENT_STATE_ARM_PREP;
                    ascent2Timer.reset();
                }
                break;

            case ASCENT_STATE_ARM_PREP:
                motorsFinished = !robot.viperMotorBusy && !robot.wormTiltMotorBusy;
                if( motorsFinished || (ascent2Timer.milliseconds() > 3000.0) ) {
                    // Rotate arm back against top bar in preparation for hooking
                    robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_LEVEL2D_DEG);
                    ascent2state = ASCENT_STATE_HOOKING;
                }
                break;

            case ASCENT_STATE_HOOKING:
                motorsFinished = !robot.wormTiltMotorBusy;
                if( motorsFinished || (ascent2Timer.milliseconds() > 3000.0) ) {
                    // Retract arm to ensure the hook engages
                    robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_LEVEL2E, 1.0, 1.0);
                    ascent2state = ASCENT_STATE_LIFTING;
                }
                break;

            case ASCENT_STATE_LIFTING:
                motorsFinished = !robot.viperMotorBusy;
                if( motorsFinished || (ascent2Timer.milliseconds() > 3000.0) ) {
                    // Retract the arm and rotate it to lift the robot off the floor
                    robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_LEVEL2F, 1.0, 1.0);
                    robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_LEVEL2F_DEG);
                    ascent2state = ASCENT_STATE_HANGING;
                }
                break;

            case ASCENT_STATE_HANGING:
                motorsFinished = !robot.viperMotorBusy && !robot.wormTiltMotorBusy;
                if( motorsFinished ) {
                    // Once snorkel is retracted and tilt has finished, we're DONE!
                    // Wait for command to safely lower
                    if(gamepad1_triangle_now && !gamepad1_triangle_last){
                        robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_LEVEL2G, 1.0, 1.0);
                        robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_LEVEL2G_DEG);
                        ascent2state = ASCENT_STATE_IDLE;
                    }
                }
                break;

        } // switch()

        if(ascent2telem) {
            // Monitor motor currents
            robot.updateAscendMotorAmps();
            telemetry.addData("Viper Motor", "%.1f Amp (%.1f peak)", 
               robot.viperMotorAmps, robot.viperMotorAmpsPk );
            telemetry.addData("Tilt Motor", "%.1f Amp (%.1f peak)",
               robot.wormTiltMotorAmps, robot.wormTiltMotorAmpsPk );
            telemetry.addData("Snorkel Motors", "%.1f %.1f Amp (%.1f %.1f peak)",
               robot.snorkleLMotorAmps, robot.snorkleLMotorAmpsPk,
               robot.snorkleRMotorAmps, robot.snorkleRMotorAmpsPk );
        } // ascent2started

    }  // processLevel2Ascent

    /*---------------------------------------------------------------------------------*/
    boolean tiltAngleCloseEnough( double tiltAngleTargetDegrees, double degreeTolerance ) {
      double tiltAngleError = robot.armTiltAngle - tiltAngleTargetDegrees;
      boolean closeEnough = (Math.abs( tiltAngleError ) <= degreeTolerance);
      return closeEnough;
    } /* tiltAngleCloseEnough */

    //************************************************************************************
    // Activity functions
    //************************************************************************************
    public void terminateAutoArmMovements() {
        abortSecureArm();
        abortHoverArm();
        abortScoreArm();
        abortScoreArmSpec();
        abortSweeper();
    }
    //**************************
    // Hover Arm - This should have the robot ready to collect, except above the level of
    //   the low bar so we can go into the pit.
    //**************************
    public enum Hover_Arm_Steps {
        IDLE,
        ROTATING_ARM,
        EXTENDING_ARM,
        POSITION_INTAKE;
    };
    public Hover_Arm_Steps hoverArmState = Hover_Arm_Steps.IDLE;
    protected ElapsedTime hoverTimer = new ElapsedTime();
    public void startHoverArm(){
        if(hoverArmState == Hover_Arm_Steps.IDLE) {
            terminateAutoArmMovements();
            robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_SUBMERSIBLE_DEG);
            robot.clawStateSet(Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE);
            hoverArmState = Hover_Arm_Steps.ROTATING_ARM;
        }
    }
    public void processHoverArm() {
        switch(hoverArmState) {
            case ROTATING_ARM:
                // Check to see if arm is in the range to start changing the viper length
                // and the intake will be ok
                if((robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_ZERO_DEG) &&
                   (robot.armTiltAngle < Hardware2025Bot.TILE_ANGLE_BASKET_SAFE_DEG)) {
                    robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_GRAB);
                    hoverArmState = Hover_Arm_Steps.EXTENDING_ARM;
                }
                break;
            case EXTENDING_ARM:
                // Check to see if the arm is out far enough to swing the intake
                if(robot.viperMotorPos > Hardware2025Bot.VIPER_EXTEND_SAFE) {
                    robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
                    robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
                    hoverTimer.reset();
                    hoverArmState = Hover_Arm_Steps.POSITION_INTAKE;
                }
                break;
            case POSITION_INTAKE:
                // Wait for the arm, slides, and intake to complete movement.
                if(!robot.viperMotorBusy && !robot.wormTiltMotorBusy && hoverTimer.milliseconds() >= 500) {
                    hoverArmState = Hover_Arm_Steps.IDLE;
                }
                break;
            case IDLE:
            default:
        }
    }
    public void abortHoverArm() {
        if(hoverArmState != Hover_Arm_Steps.IDLE) {
            robot.abortWormTilt();
            robot.abortViperSlideExtension();
            hoverArmState = Hover_Arm_Steps.IDLE;
        }
    }
    //**************************
    // Secure Arm - This should have the robot ready to run, intake up and arm nestled.
    //**************************
    public enum Secure_Arm_Steps {
        IDLE,
        ROTATING_ARM,
        EXTENDING_ARM,
        POSITION_INTAKE,
        RETRACT_ARM;

    };
    public Secure_Arm_Steps secureArmState = Secure_Arm_Steps.IDLE;
    protected ElapsedTime secureTimer = new ElapsedTime();
    public void startSecureArm(){
        if(secureArmState == Secure_Arm_Steps.IDLE) {
            terminateAutoArmMovements();
            robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_SUBMERSIBLE_DEG);
            secureArmState = Secure_Arm_Steps.ROTATING_ARM;
        }
    }
    public void processSecureArm() {
        switch(secureArmState) {
            case ROTATING_ARM:
                // Check to see if arm is in the range to start changing the viper length
                // and the intake will be ok
                if((robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_ZERO_DEG) &&
                   (robot.armTiltAngle < Hardware2025Bot.TILE_ANGLE_BASKET_SAFE_DEG)) {
                    if(robot.viperMotorPos < Hardware2025Bot.VIPER_EXTEND_SAFE)  {
                        robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_SAFE);
                    }
                    secureArmState = Secure_Arm_Steps.EXTENDING_ARM;
                }
                break;
            case EXTENDING_ARM:
                // Check to see if the arm is out far enough to swing the intake
                if(!robot.viperMotorBusy) {
                    secureTimer.reset();
                    robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_SAFE);
                    robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_SAFE);
                    secureArmState = Secure_Arm_Steps.POSITION_INTAKE;
                }
                break;
            case POSITION_INTAKE:
                // Wait for the intake to complete movement.
                if(secureTimer.milliseconds() >= 500) {
                    robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_SECURE);
                    secureArmState = Secure_Arm_Steps.RETRACT_ARM;
                }
                break;
            case RETRACT_ARM:
                if(!robot.viperMotorBusy) {
                    secureArmState = Secure_Arm_Steps.IDLE;
                }
                break;
            case IDLE:
            default:
        }
    }
    public void abortSecureArm() {
        if(secureArmState != Secure_Arm_Steps.IDLE) {
            robot.abortWormTilt();
            robot.abortViperSlideExtension();
            secureArmState = Secure_Arm_Steps.IDLE;
        }
    }
    //**************************
    // Score Arm - This should have the robot ready to score in the top bucket,
    //    arm up, rotated back, and intake in the score position.
    //**************************
    public enum Score_Arm_Steps {
        IDLE,
        ROTATING_ARM,
        RETRACTING_ARM,
        EXTENDING_ARM,
        POSITION_INTAKE;
    };
    public Score_Arm_Steps scoreArmState = Score_Arm_Steps.IDLE;
    protected ElapsedTime scoreTimer = new ElapsedTime();
    public void startScoreArm(){
        if(scoreArmState == Score_Arm_Steps.IDLE) {
            terminateAutoArmMovements();
            robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_BASKET_DEG);
            scoreArmState = Score_Arm_Steps.ROTATING_ARM;
        }
    }
    public void processScoreArm() {
        switch(scoreArmState) {
            case ROTATING_ARM:
                // Don't rotate arm over the full range of angles when FULLY EXTENDED!
                // CASE 1 - Arm past vertical (skip the temporary retracting viper to SAFE)
                // CASE 2 - Arm below vertical (retract viper to SAFE before extending to BASKET)
                if(robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_VERTICAL_DEG) {
                    scoreArmState = Score_Arm_Steps.RETRACTING_ARM;
                } else if ((robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_ZERO_DEG) &&
                           (robot.armTiltAngle < Hardware2025Bot.TILT_ANGLE_VERTICAL_DEG)) {
                    robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_SAFE);
                    scoreArmState = Score_Arm_Steps.RETRACTING_ARM;
                }
                break;
            case RETRACTING_ARM:
                // This state does two things:
                // 1) Allows the arm to retract to the SAFE position
                // 2) Waits for tilt to reach the VERTICAL angle (then start to extend to BASKET)
                if(robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_VERTICAL_DEG) {
                    robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_BASKET);
                    scoreArmState = Score_Arm_Steps.EXTENDING_ARM;
                }
                break;
            case EXTENDING_ARM:
                // This state does two things:
                // 1) Wait viper extension to BASKET to complete
                // 2) *ASSUMES* that tilt arm has reached the final angle
                // Once fully extended above the basket, flip the elbow/wrist to scoring positions
                // Start a timer (used to reset back to IDLE state)
                if(!robot.viperMotorBusy) {
                    robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BASKET);
                    robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BASKET2);
                    scoreTimer.reset();
                    scoreArmState = Score_Arm_Steps.POSITION_INTAKE;
                }
                break;
            case POSITION_INTAKE:
                if(!robot.viperMotorBusy && !robot.wormTiltMotorBusy && scoreTimer.milliseconds() >= 500) {
                    scoreArmState = Score_Arm_Steps.IDLE;
                }
                break;
            case IDLE:
            default:
        }
    }
    public void abortScoreArm() {
        if(scoreArmState != Score_Arm_Steps.IDLE) {
            robot.abortWormTilt();
            robot.abortViperSlideExtension();
            scoreArmState = Score_Arm_Steps.IDLE;
        }
    }
    //**************************
    // Score Arm Specimen - This should have the robot ready to score on the top bar,
    //    arm up, rotated back, and intake in the clip position.
    //**************************
    public enum Score_Arm_Spec_Steps {
        IDLE,
        ROTATING_ARM,
        EXTENDING_ARM,
        POSITION_INTAKE;
    };
    public Score_Arm_Spec_Steps scoreArmSpecState = Score_Arm_Spec_Steps.IDLE;
    protected ElapsedTime scoreSpecTimer = new ElapsedTime();
    public void startScoreArmSpec(){
        if(scoreArmSpecState == Score_Arm_Spec_Steps.IDLE) {
            terminateAutoArmMovements();
            robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_SPECIMEN3_DEG);
            scoreArmSpecState = Score_Arm_Spec_Steps.ROTATING_ARM;
            //robot.geckoServo.setPower(-0.3);
        }
    }
    public void processScoreArmSpec() { // REVERSE SCORING!
        switch(scoreArmSpecState) {
            case ROTATING_ARM:
                // Check to see if arm is in the range to start changing the viper length
                // and the intake will be ok
                if((robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_ZERO_DEG) &&
                   (robot.armTiltAngle < Hardware2025Bot.TILE_ANGLE_BASKET_SAFE_DEG)) {
                    robot.startViperSlideExtension(Hardware2025Bot.VIPER_EXTEND_AUTO3);
                    scoreArmSpecState = Score_Arm_Spec_Steps.EXTENDING_ARM;
                }
                break;
            case EXTENDING_ARM:
                // Check to see if the arm is out far enough to swing the intake
                if(robot.viperMotorPos > Hardware2025Bot.VIPER_EXTEND_SAFE) {
                    robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR3);
                    robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR3);
                    scoreSpecTimer.reset();
                    scoreArmSpecState = Score_Arm_Spec_Steps.POSITION_INTAKE;
                }
                break;
            case POSITION_INTAKE:
                if(!robot.viperMotorBusy && !robot.wormTiltMotorBusy && scoreSpecTimer.milliseconds() >= 500) {
                    //robot.geckoServo.setPower(0.0);
                    scoreArmSpecState = Score_Arm_Spec_Steps.IDLE;
                }
                break;
            case IDLE:
            default:
        }
    }
    public void abortScoreArmSpec() {
        if(scoreArmSpecState != Score_Arm_Spec_Steps.IDLE) {
            robot.abortWormTilt();
            robot.abortViperSlideExtension();
            scoreArmSpecState = Score_Arm_Spec_Steps.IDLE;
        }
    }
    //**************************
    // Sweeper Arm Score
    //**************************
    public enum Sweeper_Steps {
        IDLE,
        SWEEPING,
        OPENING_CLAW,
        LIFT_ARM,
        CLOSE_CLAW_LOWER_ARM,
        WIGGLE_CLAW_1,
        COLLECT_SAMPLE;
    };
    public Sweeper_Steps sweeperState = Sweeper_Steps.IDLE;
    protected ElapsedTime sweeperTimer = new ElapsedTime();

    public void startSweeper(){
        if(sweeperState == Sweeper_Steps.IDLE) {
            //Set robot into position to sweep
            terminateAutoArmMovements();
            robot.clawStateSet(Hardware2025Bot.clawStateEnum.CLAW_OPEN_SWEEPER);
            robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_SWEEPER_DEG);
            sweeperState = Sweeper_Steps.SWEEPING;
            sweeperTimer.reset();
        }
    } // startSweeper

    public void processSweeper() {
        double elapsedSweeperTime = sweeperTimer.milliseconds();
//        double tiltError = Math.abs( robot.armTiltAngle - Hardware2025Bot.TILT_ANGLE_SWEEPER_DEG);
        double tiltError = 0;
        boolean tiltReady = (tiltError < 0.25);
        boolean clawReady, movementTimeout;
        switch( sweeperState ) {
            case SWEEPING:
                clawReady = (elapsedSweeperTime > 800);
                movementTimeout = (elapsedSweeperTime > 1500);
                if( (clawReady && tiltReady) || movementTimeout ) {
                    //Bring arm down and open claw to clear samples
                    robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_SWEEPER_LOWER_DEG);
                    sweeperState = Sweeper_Steps.OPENING_CLAW;
                    sweeperTimer.reset();
                }
                break;
            case OPENING_CLAW:
                clawReady = (elapsedSweeperTime > 800);
                movementTimeout = (elapsedSweeperTime > 1500);
                if( (clawReady && tiltReady) || movementTimeout ) {
                    //Bring arm down and open claw to clear samples
                    robot.clawStateSet(Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE);
                    sweeperState = Sweeper_Steps.LIFT_ARM;
                    sweeperTimer.reset();
                }
                break;
            case LIFT_ARM:
                clawReady = (elapsedSweeperTime > 800);
                movementTimeout = (elapsedSweeperTime > 1500);
                if( (clawReady && tiltReady) || movementTimeout ) {
                    //Lift arm up then close claw to get ready to collect
                    robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_SUBMERSIBLE_DEG);
                    sweeperState = Sweeper_Steps.CLOSE_CLAW_LOWER_ARM;
                    sweeperTimer.reset();
                }
                break;
            case CLOSE_CLAW_LOWER_ARM:
                movementTimeout = (elapsedSweeperTime > 1500);
                if( tiltReady || movementTimeout ) {
                    //Close claw to narrow position and lower arm to collect position and wiggle claw
                    robot.clawStateSet(Hardware2025Bot.clawStateEnum.CLAW_OPEN_NARROW);
                    robot.startWormTilt(Hardware2025Bot.TILT_ANGLE_COLLECT_DEG);
                    sweeperState = Sweeper_Steps.WIGGLE_CLAW_1;
                    sweeperTimer.reset();
                }
                break;
            case WIGGLE_CLAW_1:
                clawReady = (elapsedSweeperTime > 800);
                movementTimeout = (elapsedSweeperTime > 1500);
                if( (clawReady && tiltReady) || movementTimeout ) {
                    //Wiggle claw to clear unwanted samples
                    //robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRABL2);
                    //robot.elbowSa ervo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRABR2);
                    robot.clawStateSet(Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE);
                    sweeperState = Sweeper_Steps.COLLECT_SAMPLE;
                    sweeperTimer.reset();
                }
                break;
            case COLLECT_SAMPLE:
                clawReady = (elapsedSweeperTime > 800);
                movementTimeout = (elapsedSweeperTime > 1500);
                if( (clawReady && tiltReady) || movementTimeout ) {
                    //Wiggle claw to clear collect sample
                    robot.clawStateSet(Hardware2025Bot.clawStateEnum.CLAW_CLOSED);
                    sweeperState = Sweeper_Steps.IDLE;
                    sweeperTimer.reset();
                }
                break;
            case IDLE:
            default:
        }
    } // processSweeper

    public void abortSweeper() {
        if(sweeperState != Sweeper_Steps.IDLE) {
            robot.abortWormTilt();
            robot.abortViperSlideExtension();
            sweeperState = Sweeper_Steps.IDLE;
        }
    } // abortSweeper

} // Teleop
