/* FTC Team 7572 - Version 2.0 (12/10/2021)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp Full Control.
 */
//@Disabled
public abstract class Teleop extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;  // Capping arm score position
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;  // Duck motor control
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;  // Capping arm claw open/close
    boolean gamepad1_square_last,     gamepad1_square_now     = false;  // Capping arm collect/store positions
//  boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;  // gamepad1.dpad_up used live/realtime
//  boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;  //   (see processDpadDriveMode() below)
//  boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
//  boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
//  boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;  // gamepad1 bumpers used live/realtime
//  boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;  //  (see processCappingArmControls() below)

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  //
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  // Freight Arm (Transport height)
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  // Freight Arm (Collect height)
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  // Intake reverse
    boolean gamepad2_dpad_up_last,    gamepad2_dpad_up_now    = false;  // Freight Arm (Hub-Top)
    boolean gamepad2_dpad_down_last,  gamepad2_dpad_down_now  = false;  // Freight Arm (Hub-Bottom) 
    boolean gamepad2_dpad_left_last,  gamepad2_dpad_left_now  = false;  // Freight Arm (Hub-Middle)
    boolean gamepad2_dpad_right_last, gamepad2_dpad_right_now = false;  // Freight Arm (score FRONT)
    boolean gamepad2_l_bumper_last,   gamepad2_l_bumper_now   = false;  // sweeper (reverse)
    boolean gamepad2_r_bumper_last,   gamepad2_r_bumper_now   = false;  // box servo (dump)

    boolean sweeperRunning  = false;  // Intake sweeper forward (fast/continuous - for collecting)
    boolean sweeperEjecting = false;  // Intake sweeper reverse (fast/continuous - eject extra freight)
    boolean sweeperDumping  = false;  // Intake sweeper reverse (slow/button - while dumping)
    boolean clawServoOpen   = false;  // true=OPEN; false=CLOSED on team element

    double    freightArmServoPos = 0.48;      // Which servo setting to target once movement starts

    final int FREIGHT_CYCLECOUNT_START = 20;  // Freight Arm just started moving (1st cycle)
    final int FREIGHT_CYCLECOUNT_SERVO = 10;  // Freight Arm off the floor (safe to rotate box servo)
    final int FREIGHT_CYCLECOUNT_CHECK = 1;   // Time to check if Freight Arm is still moving?
    final int FREIGHT_CYCLECOUNT_DONE  = 0;   // Movement is complete (cycle count is reset)
    int       freightArmCycleCount     = FREIGHT_CYCLECOUNT_DONE;
    boolean   freightArmTweaked        = false;  // Reminder to zero power when trigger released

    double    wristServoPos = 0.950;          // Servo setting to target once arm movement starts (WRIST_SERVO_INIT)

    final int CAPPING_CYCLECOUNT_START = 30;  // Capping Arm just started moving (1st cycle)
    final int CAPPING_CYCLECOUNT_SERVO = 20;  // Capping Arm off chassis (safe to rotate wrist servo)
    final int CAPPING_CYCLECOUNT_CHECK = 1;   // Time to check if Capping Arm is still moving?
    final int CAPPING_CYCLECOUNT_DONE  = 0;   // Movement is complete (cycle count is reset)
    int       cappingArmCycleCount     = CAPPING_CYCLECOUNT_DONE;
    boolean   cappingArmTweaked        = false;  // Reminder to zero power when joystick released

    double  yTranslation, xTranslation, rotation;                  /* Driver control inputs */
    double  rearLeft, rearRight, frontLeft, frontRight, maxPower;  /* Motor power levels */
    boolean backwardDriveControl = false; // drive controls backward (other end of robot becomes "FRONT")
    boolean controlMultSegLinear = true;

    final int DRIVER_MODE_STANDARD     = 2;
    final int DRIVER_MODE_DRV_CENTRIC  = 3;
    int       driverMode               = DRIVER_MODE_STANDARD;
    double    driverAngle              = 0.0;  /* for DRIVER_MODE_DRV_CENTRIC */

    boolean   duckMotorEnable = false;

    // These are set in the alliance specific teleops
    double    duckPower;
    double    duckVelocity;

    double    sonarRangeL=0.0, sonarRangeR=0.0, sonarRangeF=0.0, sonarRangeB=0.0;
    boolean   rangeSensorsEnabled = false;  // enable only when designing an Autonomous plan (takes time!)
    boolean   rangeSensorPingPong = true;   // only send a new ping out every other control cycle
    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    elapsedTime, elapsedHz;

    /* Declare OpMode members. */
    HardwareBothHubs robot = new HardwareBothHubs();

    // sets unique behavior based on alliance
    public abstract void setAllianceSpecificBehavior();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

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

            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();

            // If enabled, process ultrasonic range sensors
            if( rangeSensorsEnabled ) {
                if( rangeSensorPingPong ) {
                    processRangeSensors();
                    rangeSensorPingPong = false;
                }
                else {
                    // skip this cycle
                    rangeSensorPingPong = true;
                }
            } // rangeSensorsEnabled

            // Process all the driver/operator inputs
            processDuckMotorControls();
            processFreightArmControls();
            processSweeperControls();
            processCappingArmControls();

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
            telemetry.addData("Back ", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
                    rearLeft,  robot.rearLeftMotorVel,  rearRight,  robot.rearRightMotorVel );
            telemetry.addData("Duck ", "%.2f (%.0f cts/sec)",
                    duckPower,  robot.duckMotorVel );
            telemetry.addData("Freight Arm", "%d cts %.2f mA", robot.freightMotorPos, robot.freightMotorAmps );
            telemetry.addData("Capping Arm", "%d cts %.2f mA", robot.cappingMotorPos, robot.cappingMotorAmps );
            telemetry.addData("Capping Wrist", "%.3f (commanded)", robot.wristServo.getPosition() );
            if( rangeSensorsEnabled ) {
               telemetry.addData("Sonar Range (L/R)", "%.1f  %.1f in", sonarRangeL/2.54, sonarRangeR/2.54 );
               telemetry.addData("Sonar Range (F/B)", "%.1f  %.1f in", sonarRangeF/2.54, sonarRangeB/2.54 );
            }
//          telemetry.addData("Gyro Angle", "%.1f deg", robot.headingIMU() );
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
    } // captureGamepad2Buttons

    /*---------------------------------------------------------------------------------*/
    void processSweeperControls() {
        // Check if gamepad2 LEFT BUMPER is pressed
        if( gamepad2_l_bumper_now ) {
            robot.sweepServo.setPower( -0.15 );  // ON (reverse)
            sweeperDumping = true;   // note that we need to turn it back OFF
        }
        // or not. but was PREVIOUSLY
        else if( sweeperDumping ) {
            robot.sweepServo.setPower( 0.0 );  // OFF
            sweeperDumping = false;   // only do this once!
        }
        // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button
        if( gamepad2_square_now && !gamepad2_square_last) {
          if( sweeperEjecting ) {  // already reverse; toggle back to forward
            robot.sweepServo.setPower( 1.0 );  // ON (forward)
            sweeperRunning  = true;
            sweeperEjecting = false;
          }
          else {  // currently forward, so switch to reverse
            robot.sweepServo.setPower( -1.0 );  // ON (reverse)
            sweeperRunning  = true;
            sweeperEjecting = true;
          }
        } // square
        // Check for an OFF-to-ON toggle of the gamepad1 TRIANGLE button
        if( gamepad2_triangle_now && !gamepad2_triangle_last) {
            robot.sweepServo.setPower( 0.0 );  // OFF
            sweeperRunning  = false;
            sweeperEjecting = false;
        } // triangle
    } // processSweeperControls

    /*---------------------------------------------------------------------------------*/
    void processDuckMotorControls() {
        // Check for an OFF-to-ON toggle of the gamepad1 CIRCLE button
        if( gamepad1_circle_now && !gamepad1_circle_last)
        {   // toggle motor ON/OFF
            if( duckMotorEnable ) {
//              robot.duckMotor.setPower( 0.0 );
                robot.duckMotor.setVelocity( 0.0 );
            }
            else {
//              robot.duckMotor.setPower( duckPower );
                robot.duckMotor.setVelocity( duckVelocity );
            }
            duckMotorEnable = !duckMotorEnable;
        }
    } // processDuckMotorControls

    /*---------------------------------------------------------------------------------*/
    double determineFreightArmPos() {
        double servoTarget   = robot.BOX_SERVO_DUMP_BOTTOM; // updated below...
        int    freightArmPos = robot.freightMotorPos;       // current encoder count
        int    midpoint1     = (robot.FREIGHT_ARM_POS_HUB_TOP    + robot.FREIGHT_ARM_POS_HUB_MIDDLE)/2;
        int    midpoint2     = (robot.FREIGHT_ARM_POS_HUB_MIDDLE + robot.FREIGHT_ARM_POS_HUB_BOTTOM)/2;
        // Determine servo DUMP ANGLE based on  freight-arm location 
        if( freightArmPos < robot.FREIGHT_ARM_POS_VERTICAL )
            servoTarget = robot.BOX_SERVO_DUMP_FRONT;
        else if( freightArmPos < midpoint1 )
            servoTarget = robot.BOX_SERVO_DUMP_TOP;
        else if( freightArmPos < midpoint2 )
            servoTarget = robot.BOX_SERVO_DUMP_MIDDLE;
        else 
            servoTarget = robot.BOX_SERVO_DUMP_BOTTOM;
        
        return servoTarget;
    } // determineFreightArmPos

    /*---------------------------------------------------------------------------------*/
    void processFreightArmControls() {
        // Check for an OFF-to-ON toggle of the gamepad2 RIGHT BUMPER
        if( gamepad2_r_bumper_now && !gamepad2_r_bumper_last)
        {
            double boxServoTarget = determineFreightArmPos();
            robot.boxServo.setPosition( boxServoTarget );   // DUMP!
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 CIRCLE button
        if( gamepad2_circle_now && !gamepad2_circle_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_TRANSPORT;
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_TRANSPORT1, 0.80 );
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
            // automatically turn OFF the sweeper
            robot.sweepServo.setPower( 0.0 );  // OFF
            sweeperRunning = false;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 CROSS button ()
        if( gamepad2_cross_now && !gamepad2_cross_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_COLLECT;
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_COLLECT, 0.20 );
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
            // automatically turn ON the sweeper
            robot.sweepServo.setPower( 1.0 );  // ON
            sweeperRunning = true;
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD UP
        if( gamepad2_dpad_up_now && !gamepad2_dpad_up_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_TRANSPORT;
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_TOP, 0.85 );
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD LEFT
        if( gamepad2_dpad_left_now && !gamepad2_dpad_left_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_TRANSPORT;
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_MIDDLE, 0.85 );
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD RIGHT
        if( gamepad2_dpad_right_now && !gamepad2_dpad_right_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_TRANSPORT;
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_SHARED, 0.50 );
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD DOWN
        if( gamepad2_dpad_down_now && !gamepad2_dpad_down_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_TRANSPORT;
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_BOTTOM, 0.85 );
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        //===================================================================
        if( freightArmCycleCount > FREIGHT_CYCLECOUNT_SERVO ) {
            // nothing to do yet (just started moving)
            freightArmCycleCount--;
        }
        else if( freightArmCycleCount == FREIGHT_CYCLECOUNT_SERVO ) {
            robot.boxServo.setPosition( freightArmServoPos );
            freightArmCycleCount--;
        }
        else if( freightArmCycleCount > FREIGHT_CYCLECOUNT_CHECK ) {
            // nothing to do yet (too soon to check motor state)
            freightArmCycleCount--;
        }
        else if( freightArmCycleCount == FREIGHT_CYCLECOUNT_CHECK ) {
            if( robot.freightMotor.isBusy() ) {
                // still moving; hold at this cycle count
            }
            else { // no longer busy; turn off motor power
                robot.freightMotor.setPower( 0.0 );
                robot.freightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                freightArmCycleCount = FREIGHT_CYCLECOUNT_DONE;   // ensure we're reset
            }
        }
        else { // freightArmCycleCount == FREIGHT_CYCLECOUNT_DONE
            // arm must be idle; check for manual arm control
            freightArmCycleCount = FREIGHT_CYCLECOUNT_DONE;   // ensure we're reset
            if( gamepad2.left_stick_y > 0.05 ) {
                robot.freightMotor.setPower( 0.20 );
                freightArmTweaked = true;
            }
            else if( gamepad2.left_stick_y < -0.05 ) {
                robot.freightMotor.setPower( -0.20 );
                freightArmTweaked = true;
            }
            else if( freightArmTweaked ) {
                robot.freightMotor.setPower( 0.0 );
                freightArmTweaked = false;
            }
        }
    } // processFreightArmControls

    /*---------------------------------------------------------------------------------*/
    void processCappingArmControls() {
        // Check for an OFF-to-ON toggle of the gamepad1 CROSS button
        if( gamepad1_cross_now && !gamepad1_cross_last)
        {
            if( clawServoOpen ) {
                robot.clawServo.setPosition( robot.CLAW_SERVO_CLOSED );
            }
            else {
                robot.clawServo.setPosition( robot.CLAW_SERVO_OPEN );
            }
            clawServoOpen = !clawServoOpen;
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 TRIANGLE button
        if( gamepad1_triangle_now && !gamepad1_triangle_last)
        {
            // <MIN> STORE ... midpoint1 ... CAP ... midpoint2 ... GRAB <MAX>
            int midpoint1 = (robot.CAPPING_ARM_POS_STORE + robot.CAPPING_ARM_POS_CAP)/2;
            int midpoint2 = (robot.CAPPING_ARM_POS_CAP   + robot.CAPPING_ARM_POS_GRAB)/2;
            // toggle into and out of CAP position (use current arm position to decide)
            if( (robot.cappingMotorPos < midpoint1) ||   /* currently STORE */
                (robot.cappingMotorPos > midpoint2) )    /* currently GRAB  */
            {  // switch to CAP
            wristServoPos =  robot.WRIST_SERVO_CAP;
            robot.cappingArmPosition( robot.CAPPING_ARM_POS_CAP, 0.70 );
            cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
            else
            { // currently CAP; switch to STORE
              wristServoPos = robot.WRIST_SERVO_STORE;
              robot.cappingArmPosition( robot.CAPPING_ARM_POS_STORE, 0.70 );
              cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button
        else if( gamepad1_square_now && !gamepad1_square_last)
        {
            // toggle between GRAB and STORE positions 
            // (use current arm position to decide)
            if( robot.cappingMotorPos < robot.CAPPING_ARM_POS_CAP )
            {  // currently STORE; switch to GRAB
              wristServoPos = robot.WRIST_SERVO_GRAB;
              robot.cappingArmPosition( robot.CAPPING_ARM_POS_GRAB, 0.70 );
              cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
            else
            { // currently GRAB; switch to STORE
              wristServoPos = robot.WRIST_SERVO_STORE;
              robot.cappingArmPosition( robot.CAPPING_ARM_POS_STORE, 0.70 );
              cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
        }
        //===================================================================
        if( cappingArmCycleCount > CAPPING_CYCLECOUNT_SERVO ) {
            // nothing to do yet (just started moving)
            cappingArmCycleCount--;
        }
        else if( cappingArmCycleCount == CAPPING_CYCLECOUNT_SERVO ) {
            robot.wristServo.setPosition( wristServoPos );
            cappingArmCycleCount--;
        }
        else if( cappingArmCycleCount > CAPPING_CYCLECOUNT_CHECK ) {
            // nothing to do yet (too soon to check motor state)
            cappingArmCycleCount--;
        }
        else if( cappingArmCycleCount == CAPPING_CYCLECOUNT_CHECK ) {
            if( robot.cappingMotor.isBusy() ) {
                // still moving; hold at this cycle count
            }
            else { // no longer busy; turn off motor power
                robot.cappingMotor.setPower( 0.0 );
                robot.cappingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                cappingArmCycleCount = CAPPING_CYCLECOUNT_DONE;   // ensure we're reset
            }
        }
        else { // cappingArmCycleCount == CAPPING_CYCLECOUNT_DONE
            // arm must be idle; check for manual arm control
            cappingArmCycleCount = CAPPING_CYCLECOUNT_DONE;   // ensure we're reset
            double gamepad1_left_trigger  = gamepad1.left_trigger;
            double gamepad1_right_trigger = gamepad1.right_trigger;
            if( gamepad1_left_trigger > 0.05 ) {
                // limit how far we can drive this direction
                if( robot.cappingMotorPos < robot.CAPPING_ARM_POS_GRAB ) {
                    robot.cappingMotor.setPower( +0.10 * gamepad1_left_trigger );
                    cappingArmTweaked = true;
                }
                else {
                    robot.cappingMotor.setPower( 0.0 );
                }
            }
            else if( gamepad1_right_trigger > 0.05 ) {
                // limit how far we can drive this direction
                if( robot.cappingMotorPos > 0 ) {
                    robot.cappingMotor.setPower( -0.10 * gamepad1_right_trigger );
                    cappingArmTweaked = true;
                }
                else {
                    robot.cappingMotor.setPower( 0.0 );
                }
            }
            else if( cappingArmTweaked ) {
                robot.cappingMotor.setPower( 0.0 );
                cappingArmTweaked = false;
            }
        }
        //===================================================================
        if( gamepad1.left_bumper ) {
            // What was the last commanded position?
            double curPos = robot.wristServo.getPosition();
            if( curPos >  -0.95 ) {
                double newPos = curPos - 0.005;
                robot.wristServo.setPosition( newPos );
            }
        }
        else if( gamepad1.right_bumper ) {
            // What was the last commanded position?
            double curPos = robot.wristServo.getPosition();
            if( curPos <  0.95 ) {
                double newPos = curPos + 0.005;
                robot.wristServo.setPosition( newPos );
            }
        }
    } // processCappingArmControls

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Mecanum-wheel drive control using Dpad (slow/fine-adjustment mode)    */
    /*---------------------------------------------------------------------------------*/
    boolean processDpadDriveMode() {
        double fineControlSpeed = 0.15;
        double fineTurnSpeed    = 0.05;
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
/* DISABLE D-PAD LEFT/RIGHT STRAFE FOR THIS SEASON
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
*  INSTEAD USE LEFT/RIGHT FOR FINE-TURNING CONTROL */
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
//          yTranslation = multSegLinearXY( -gamepad1.left_stick_y );
//          xTranslation = multSegLinearXY(  gamepad1.left_stick_x );
//          rotation     = multSegLinearRot( -gamepad1.right_stick_x );
            yTranslation = -gamepad1.left_stick_y * 0.60;
            xTranslation =  gamepad1.left_stick_x * 0.60;
            rotation     = -gamepad1.right_stick_x * 0.28;
        }
        else {
            yTranslation = -gamepad1.left_stick_y;
            xTranslation = gamepad1.left_stick_x;
            rotation = -gamepad1.right_stick_x;
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
    void processRangeSensors() {
        sonarRangeL = robot.updateSonarRangeL();
        sonarRangeR = robot.updateSonarRangeR();
        sonarRangeF = robot.updateSonarRangeF();
        sonarRangeB = robot.updateSonarRangeB();
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
          sonarRangeR = robot.updateSonarRangeR();
          sonarRangeF = robot.updateSonarRangeF();
          sonarRangeB = robot.updateSonarRangeB();
        }
    } // averagedRangeSensors

} // TeleopBlue
