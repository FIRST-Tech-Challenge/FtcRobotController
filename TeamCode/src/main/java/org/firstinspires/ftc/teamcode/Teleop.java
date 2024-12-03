/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    int       driverMode               = DRIVER_MODE_STANDARD;
//  int       driverMode               = DRIVER_MODE_DRV_CENTRIC;
    double    driverAngle              = 0.0;  /* for DRIVER_MODE_DRV_CENTRIC */

    boolean   batteryVoltsEnabled = false;  // enable only during testing (takes time!)

    double    viperPower = 0.0;
    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    elapsedTime, elapsedHz;

    /* Declare OpMode members. */
    Hardware2025Bot robot = new Hardware2025Bot();

    boolean leftAlliance = true;  // overriden in setAllianceSpecificBehavior()
    boolean blueAlliance = true;  // overriden in setAllianceSpecificBehavior()

    int     aprilTagHuman  = 11;  // overriden in setAllianceSpecificBehavior() Default to Blue Alliance
    int     aprilTagStart  = 12;  // overriden in setAllianceSpecificBehavior() Default to Blue Alliance
    int     aprilTagBasket = 13;  // overriden in setAllianceSpecificBehavior() Default to Blue Alliance

    boolean geckoServoCollecting = false; // Is the collector servo currently intaking (true) or OFF (false);
    boolean geckoServoEjecting   = false; // Is the collector servo currently ejecting (true) or OFF (false);
    boolean panAngleTweaked      = false; // Reminder to zero power when PAN  input stops
    boolean tiltAngleTweaked     = false; // Reminder to zero power when TILT input stops
    boolean liftTweaked          = false; // Reminder to zero power when LIFT input stops
    boolean enableOdometry       = true; // Process/report odometry updates?

    double   curX, curY, curAngle;
    double   minX=0.0, maxX=0.0, minY=0.0, maxY=0.0;

    final int   ASCENT_STATE_IDLE   = 0;
    final int   ASCENT_STATE_SETUP  = 1;
    final int   ASCENT_STATE_MOVING = 2;
    final int   ASCENT_STATE_READY  = 3;
    final int   ASCENT_STATE_LEVEL2 = 4;

    int         ascent2state = ASCENT_STATE_IDLE;
	boolean     ascent2telem = false;

    int geckoWheelState = 0;
	
    Gamepad.RumbleEffect visibleAprilTagRumble1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect rumblePixelBinSingle;
    Gamepad.RumbleEffect rumblePixelBinDouble;

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

        rumblePixelBinDouble = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 250)  //  Rumble RIGHT motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(0.0, 1.0, 250)  //  Rumble RIGHT motor 100% for 250 mSec
                .build();

        // Initialize robot hardware (not autonomous mode)
        robot.init(hardwareMap,false);

        setAllianceSpecificBehavior();

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
                curX     = pos.getX(DistanceUnit.INCH);  if(curX<minX){minX=curX;} if(curX>maxX){maxX=curX;}
                curY     = pos.getY(DistanceUnit.INCH);  if(curY<minY){minY=curY;} if(curY>maxY){maxY=curY;}
                curAngle = pos.getHeading(AngleUnit.DEGREES);
                String posStr = String.format(Locale.US, "{X,Y: %.1f, %.1f in  H: %.1f deg}", curX, curY, curAngle);
                telemetry.addData("Position", posStr);
                telemetry.addData("Odo Circle", "x=%.1f, y=%.1f inches", (maxX-minX), (maxY-minY) );
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

//          processPanAndTilt();
            processPanControls();
            processTiltControls();
            ProcessViperLiftControls();
            processCollectorControls();
//          processLevel2Ascent();
            performEveryLoopTeleop();

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            elapsedTime  = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            elapsedHz    =  1000.0 / elapsedTime;

            // Update telemetry data
//          telemetry.addData("Front", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
//                  frontLeft, robot.frontLeftMotorVel, frontRight, robot.frontRightMotorVel );
//          telemetry.addData("Rear ", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
//                  rearLeft,  robot.rearLeftMotorVel,  rearRight,  robot.rearRightMotorVel );
//          telemetry.addData("Front", "%d %d counts", robot.frontLeftMotorPos, robot.frontRightMotorPos );
//          telemetry.addData("Back ", "%d %d counts", robot.rearLeftMotorPos,  robot.rearRightMotorPos );
            telemetry.addData("Pan", "%d counts", robot.wormPanMotorPos );
            telemetry.addData("Tilt", "%d counts %.1f deg %.1f raw deg", robot.wormTiltMotorPos, robot.armTiltAngle, robot.computeRawAngle(robot.armTiltEncoder.getVoltage()));
            telemetry.addData("Viper", "%d counts", robot.viperMotorPos );
            telemetry.addData("Elbow", "%.1f (%.1f deg)", robot.getElbowServoPos(), robot.getElbowServoAngle() );
            telemetry.addData("Wrist", "%.1f (%.1f deg)", robot.getWristServoPos(), robot.getElbowServoAngle() );
//          telemetry.addData("Gyro Angle", "%.1f degrees", robot.headingIMU() );
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", elapsedTime, elapsedHz );
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//          robot.waitForTick(40);
        } // opModeIsActive

    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void performEveryLoopTeleop() {
       robot.processViperSlideExtension();        
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
//      gamepad2_touchpad_last   = gamepad2_touchpad_now;    gamepad2_touchpad_now   = gamepad2.touchpad;
//      gamepad2_share_last      = gamepad2_share_now;       gamepad2_share_now      = gamepad2.share;
    } // captureGamepad2Buttons

    /*  TELE-OP: Mecanum-wheel drive control using Dpad (slow/fine-adjustment mode)    */
    /*---------------------------------------------------------------------------------*/
    boolean processDpadDriveMode() {
        double fineControlSpeed = 0.30;
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
            if( valueIn < 0.33 ) {                      // NOTE: approx 0.06 requfired to **initiate** rotation
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
    void processPanAndTilt() {

        // Pan movement
        if(Math.abs(gamepad2.left_stick_x) > 0.20) {
            double panPower = (gamepad2.left_stick_x > 0.0)? 0.10 : -0.10;
            robot.wormPanMotor.setPower(panPower);
        }
        else {
            robot.wormPanMotor.setPower(0.0);
        }

        // Tilt movement
        if(Math.abs(gamepad2.left_stick_y) > 0.05) {
            double tiltPower = (gamepad2.left_stick_y > 0.0)? 0.30 : -0.30;
            robot.wormTiltMotor.setPower(tiltPower);
        }
        else {
            robot.wormTiltMotor.setPower(0.0);
        }
    } // processPanAndTilt

    /*---------------------------------------------------------------------------------*/
    void processPanControls() {
        boolean safeToManuallyLeft  = (robot.wormPanMotorPos > robot.PAN_ANGLE_HW_MIN);
        boolean safeToManuallyRight = (robot.wormPanMotorPos < robot.PAN_ANGLE_HW_MAX);
        double  gamepad2_left_stick = gamepad2.left_stick_x;
        boolean manual_pan_control = ( Math.abs(gamepad2_left_stick) > 0.15 );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 LEFT BUMPER
        if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last )
        {
        }
        // Check for an OFF-to-ON toggle of the gamepad1 RIGHT BUMPER
        else if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last )
        {
        }

        //===================================================================
        else if( manual_pan_control || panAngleTweaked) {
            // Does user want to rotate turret LEFT (negative joystick input)
            if( safeToManuallyLeft && (gamepad2_left_stick < -0.15) ) {
                double motorPower = 0.20 * gamepad2_left_stick; // NEGATIVE
                robot.wormPanMotor.setPower( motorPower );   // -3% to -20%
                panAngleTweaked = true;
            }
            // Does user want to rotate turret RIGHT (positive joystick input)
            else if( safeToManuallyRight && (gamepad2_left_stick > 0.15) ) {
                double motorPower = 0.20 * gamepad2_left_stick; // POSITIVE
                robot.wormPanMotor.setPower( motorPower );   // +3% to +20%
                panAngleTweaked = true;
            }
            // No more input?  Time to stop turret movement!
            else if(panAngleTweaked) {
                robot.wormPanMotor.setPower( 0.0 );
                panAngleTweaked = false;
            }
        } // manual_pan_control

    } // processPanControls

    /*---------------------------------------------------------------------------------*/
    void processTiltControls() {
        // The encoder is backwards from our definition of MAX and MIN. Maybe change the
        // convention in hardware class?
        boolean safeToManuallyLower = (robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_HW_MIN_DEG);
        boolean safeToManuallyRaise = (robot.armTiltAngle < Hardware2025Bot.TILT_ANGLE_HW_MAX_DEG);
        double  gamepad2_right_stick = gamepad2.right_stick_y;
        boolean manual_tilt_control = ( Math.abs(gamepad2_right_stick) > 0.08 );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 CROSS button
        if( gamepad1_cross_now && !gamepad1_cross_last)
        {
            // robot.turretPIDPosInit( robot.PAN_ANGLE_CENTER );
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 LEFT BUMPER
        else if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last )
        {
        }

        //===================================================================
        else if( manual_tilt_control || tiltAngleTweaked) {
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
    void ProcessViperLiftControls() {
        boolean safeToManuallyRetract = (robot.viperMotorPos > robot.VIPER_EXTEND_ZERO);
        boolean safeToManuallyExtend  = (robot.viperMotorPos < robot.VIPER_EXTEND_FULL1);
        // Capture user inputs ONCE, in case they change during processing of this code
        // or we want to scale them down
        double  gamepad2_left_trigger  = gamepad2.left_trigger  * 1.00;
        double  gamepad2_right_trigger = gamepad2.right_trigger * 1.00;
        boolean manual_lift_control = ( (gamepad2_left_trigger  > 0.25) || (gamepad2_right_trigger > 0.25) );

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD UP
        if( gamepad2_dpad_up_now && !gamepad2_dpad_up_last)
        {   // Extend lift to the basket-scoring position
            robot.startViperSlideExtension( robot.VIPER_EXTEND_BASKET );
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_SAFE);
//          robot.elbowServo.setPosition(robot.ELBOW_SERVO_DROP);
            robot.wristServo.setPosition(robot.WRIST_SERVO_SAFE);
//          robot.wristServo.setPosition(robot.WRIST_SERVO_DROP);
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD RIGHT
        else if( gamepad2_dpad_right_now && !gamepad2_dpad_right_last)
        {   // Extend lift to the specimen-scoring hook-above-the-bar height
            robot.startViperSlideExtension( robot.VIPER_EXTEND_AUTO1 );
            // Position for scoring a specimen on the submersible bar
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_BAR2);
            robot.wristServo.setPosition(robot.WRIST_SERVO_BAR2);
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD DOWN
        else if( gamepad2_dpad_down_now && !gamepad2_dpad_down_last)
        {   // Retract lift to the collection position
            boolean armRaised = (robot.armTiltAngle > Hardware2025Bot.TILT_ANGLE_RAISED_DEG)? true: false;
            // If raised to the basket, do this automatically before lowering
            // (don't want to do it if we use this backing out of submersimble
            if (armRaised) {
                robot.elbowServo.setPosition(robot.ELBOW_SERVO_GRAB);
                robot.wristServo.setPosition(robot.WRIST_SERVO_GRAB);
                sleep(250);
            }
            robot.startViperSlideExtension( robot.VIPER_EXTEND_GRAB );
        }
        //===================================================================
        else if( manual_lift_control || liftTweaked ) {
            // Does user want to manually RAISE the lift?
            if( safeToManuallyExtend && (gamepad2_right_trigger > 0.25) ) {
                // Do we need to terminate an auto movement?
                robot.abortViperSlideExtension();
                viperPower = gamepad2_right_trigger;
                robot.viperMotor.setPower( viperPower );  // fixed power? (robot.VIPER_RAISE_POWER)
                liftTweaked = true;
            }
            // Does user want to manually LOWER the lift?
            else if( safeToManuallyRetract && (gamepad2_left_trigger > 0.25) ) {
                // Do we need to terminate an auto movement?
                robot.abortViperSlideExtension();
                viperPower = robot.VIPER_LOWER_POWER;
                robot.viperMotor.setPower( viperPower );
                liftTweaked = true;
            }
            // No more input?  Time to stop lift movement!
            else if( liftTweaked ) {
                // if the lift is near the bottom, truly go to zero power
                // but if in a raised position, only drop to minimal holding power
                boolean closeToZero = (Math.abs(robot.viperMotorPos - robot.VIPER_EXTEND_ZERO) < 20);
                viperPower = closeToZero? 0.0 : robot.VIPER_HOLD_POWER;
                robot.viperMotor.setPower( viperPower );
                liftTweaked = false;
            }
        } // manual_lift_control

    }  // ProcessLiftControls

    /*---------------------------------------------------------------------------------*/
    void processCollectorControls() {
        // Check for an OFF-to-ON toggle of the gamepad2 CIRCLE button
        // - rotates the wrist/elbow to the floor collection orientation
        // TO DO: check tilt motor for safe height above floor for wrist rotation!
        if( gamepad2_circle_now && !gamepad2_circle_last)
        {
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_GRAB);
            robot.wristServo.setPosition(robot.WRIST_SERVO_GRAB);
        }
        // Check for an OFF-to-ON toggle of the gamepad2 CROSS button
        // - rotates the wrist/elbow to the horizontal transport position
        // TO DO: check tilt motor for safe height above floor for wrist rotation!
        if( gamepad2_cross_now && !gamepad2_cross_last)
        {
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_SAFE);
            robot.wristServo.setPosition(robot.WRIST_SERVO_SAFE);
        }
        // Check for an OFF-to-ON toggle of the gamepad2 SQUARE button
        // - rotates the wrist/elbow to the vertical init position
        if( gamepad2_square_now && !gamepad2_square_last )
        {
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_INIT);
            robot.wristServo.setPosition(robot.WRIST_SERVO_INIT);
        }
        // Check for an OFF-to-ON toggle of the gamepad2 left or right bumpers
        // - right enables the collector intake servo in FORWARD/collect mode
        // - left enables the collector intake servo in REVERSE/eject/score mode
        if( gamepad2_l_bumper_now && !gamepad2_l_bumper_last)
        {
            if( geckoServoEjecting ) {
               robot.geckoServo.setPower( 0.0 );  // toggle eject OFF
               geckoServoEjecting = false;
            } else /* not ejecting */ {
               robot.geckoServo.setPower( 1.0 );  // toggle eject ON
               geckoServoEjecting = true;
            }
            geckoServoCollecting = false;      // (we can't be doing this)
        } // l_bumper
        else if( gamepad2_r_bumper_now && !gamepad2_r_bumper_last)
        {
            if( geckoServoCollecting ) {
               robot.geckoServo.setPower( 0.0 );  // toggle collect OFF
               geckoServoCollecting = false;
            } else /* not collecting */ {
               robot.geckoServo.setPower( -1.0 ); // toggle collect ON
               geckoServoCollecting = true;
            }
            geckoServoEjecting = false;           // (we can't be doing this)
        } // r_bumper

    }  // processCollectorControls

    /*---------------------------------------------------------------------------------*/
    void processLevel2Ascent() {

        // DRIVER 1 controls position the arm for hanging
        // DRIVER 2 controls initiate the actual hang

        // Check for emergency ASCENT ABORT button
        if( gamepad1_touchpad_now && !gamepad1_touchpad_last ) {
            robot.viperMotor.setPower( 0.0 );
            robot.wormTiltMotor.setPower( 0.0 );
            robot.wormPanMotor.setPower( 0.0 );
            ascent2state = ASCENT_STATE_IDLE;
			ascent2telem = false;
        }

        switch( ascent2state ) {
            case ASCENT_STATE_IDLE :
                // First instance of BOTH gamepad1 left/right bumpers initiates ascent prep
                if( gamepad1_l_bumper_now && gamepad1_r_bumper_now )
                {
                    robot.wristServo.setPosition(robot.WRIST_SERVO_INIT);
                    robot.elbowServo.setPosition(robot.ELBOW_SERVO_INIT);
                    ascent2telem = true; // start monitoring motor powers
                    ascent2state = ASCENT_STATE_SETUP;
                }
                break;
            case ASCENT_STATE_SETUP:
                // Send TILT motor to hang position
                robot.wormTiltMotor.setTargetPosition( robot.computeEncoderCountsFromAngle(Hardware2025Bot.TILT_ANGLE_HANG1_DEG) );
                robot.wormTiltMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );
                robot.wormTiltMotor.setPower( 0.90 );
                // Send LIFT motor to hang position
                robot.viperMotor.setTargetPosition( robot.VIPER_EXTEND_HANG1 );
                robot.viperMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );
                robot.viperMotor.setPower( 0.90 );
                ascent2state = ASCENT_STATE_MOVING;
                break;
            case ASCENT_STATE_MOVING :
                boolean tiltReady  = (Math.abs(robot.armTiltAngle - robot.TILT_ANGLE_HANG1_DEG) < 0.2)? true:false;
                boolean viperReady = (Math.abs(robot.viperMotorPos - robot.VIPER_EXTEND_HANG1) < 20)? true:false;
                break;
            case ASCENT_STATE_READY :
                break;

            case ASCENT_STATE_LEVEL2 :
                break;
        } // switch()

        if(ascent2telem) {
            // Monitor motor currents
            robot.updateAscendMotorAmps();
            telemetry.addData("Viper Motor", "%.1f Amp (%.1f peak)", 
               robot.viperMotorAmps, robot.viperMotorAmpsPk );
            telemetry.addData("Tilt Motor", "%.1f Amp (%.1f peak)",
               robot.wormTiltMotorAmps, robot.wormTiltMotorAmpsPk );
            telemetry.addData("Pan Motor", "%.1f Amp (%.1f peak)", 
               robot.wormPanMotorAmps, robot.wormPanMotorAmpsPk );
            // Do we need to further retract viper slide motor?            
            boolean viperRetractionIncomplete = (robot.viperMotorPos > robot.VIPER_EXTEND_HANG2)? true : false;
            boolean viperMotorLoadOkay = (robot.viperMotorAmps < 8.5)? true : false;
            if( viperRetractionIncomplete && viperMotorLoadOkay ) {
                robot.viperMotor.setPower( -1.0 );
            } else {
                robot.viperMotor.setPower( -0.10 );  // hold power
            }
            // Do we need to further store the robot drivetrain?
            boolean tiltRetractionIncomplete = (robot.armTiltAngle < Hardware2025Bot.TILT_ANGLE_HANG2_DEG)? true : false;
            boolean tiltMotorLoadOkay = (robot.wormTiltMotorAmps < 8.5)? true : false;
            if( viperRetractionIncomplete && viperMotorLoadOkay ) {
                robot.wormTiltMotor.setPower( 0.30 );   // test at lower  power!!!
            } else {
                robot.wormTiltMotor.setPower( 0.0 );  // worm reverse torque holds motor
            }
        } // ascent2started

    }  // processLevel2Ascent

} // TeleopLift
