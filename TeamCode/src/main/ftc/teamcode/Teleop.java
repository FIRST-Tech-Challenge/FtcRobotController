/* FTC Team 7572 - Version 3.0 (03/26/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareBothHubs.TurretPosition;

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
    boolean gamepad1_touchpad_last,   gamepad1_touchpad_now   = false;  // autodrive to shared hub

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  //
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  // Freight Arm (Transport height)
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  // Freight Arm (Collect height)
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  // Intake reverse
    boolean gamepad2_dpad_up_last,    gamepad2_dpad_up_now    = false;  // Freight Arm (Hub-Top)
    boolean gamepad2_dpad_down_last,  gamepad2_dpad_down_now  = false;  // Freight Arm (Hub-Bottom)
    boolean gamepad2_dpad_left_last,  gamepad2_dpad_left_now  = false;  // Freight Arm (Hub-Middle)
    boolean gamepad2_dpad_right_last, gamepad2_dpad_right_now = false;  // Freight Arm (score FRONT)
    boolean gamepad2_l_bumper_last,   gamepad2_l_bumper_now   = false;  // (unused)
    boolean gamepad2_r_bumper_last,   gamepad2_r_bumper_now   = false;  // box servo (dump)
    boolean gamepad2_touchpad_last,   gamepad2_touchpad_now   = false;  // UNUSED
    boolean gamepad2_share_last,      gamepad2_share_now      = false;  // PROBLEM!! reset freight arm controls

//  double  sweeperVelocity = 1900.0; // power 1.0 = 2200 counts/sec
    double  sweeperPower    = 0.85;   // RUN_WITHOUT_ENCODER power setting that achieves 1900 cts/sec
    boolean sweeperRunning  = false;  // Intake sweeper forward (fast/continuous - for collecting)
    boolean sweeperEjecting = false;  // Intake sweeper reverse (fast/continuous - eject extra freight)
    boolean clawServoOpen   = false;  // true=OPEN; false=CLOSED on team element

    int       freightArmTarget   = 0;         // Which arm position (encoder counts) to target
    double    freightArmServoPos = 0.0;       // Which servo setting to target once movement starts

    final int FREIGHT_CYCLECOUNT_START  = 20; // Freight Arm just started moving (1st cycle)
    final int FREIGHT_ARM_START_ROTATING_TURRET = 19; // Start the freight arm centering servo
    final int FREIGHT_CYCLECOUNT_SERVO  = 10; // Freight Arm off the floor (safe to rotate box servo)
    final int FREIGHT_CYCLECOUNT_CHECK  = 3;  // Time to check if Freight Arm is still moving?
    final int FREIGHT_ARM_CENTERING     = 2;  // Center the freight arm after performing evasive maneuvers
    final int FREIGHT_CYCLECOUNT_SETTLE = 1;  // Small delay to make sure things aren't bouncing around
    final int FREIGHT_CYCLECOUNT_DONE   = 0;  // Movement is complete (cycle count is reset)
    int       freightArmCycleCount      = FREIGHT_CYCLECOUNT_DONE;
    boolean   freightArmElevTweaked     = false;  // Reminder to zero power when trigger released

    ElapsedTime freightArmDelayTimer = new ElapsedTime();
    boolean     waitForDumpServo     = false;

    double      turretStepSize       = 0.005;
    ElapsedTime turretDelayTimer     = new ElapsedTime();
    boolean     waitForTurretServo   = false;
    double      waitForTurretMsec    = 0.0; // wait time (msec)
    boolean     autoRotateTurret     = false; // Rotate toward shared hub
    boolean     waitingForTurret     = false; // Need to wait for turret to center

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
//  int       driverMode               = DRIVER_MODE_DRV_CENTRIC;
    double    driverAngle              = 0.0;  /* for DRIVER_MODE_DRV_CENTRIC */
    boolean   autoDrive                = false;

    boolean   duckMotorEnable = false;

    // collector arm variables
    boolean     collectorArmRaised   = true;   // TRUE=fully raised; FALSE=fully lowered
    boolean     needCollectorRaised  = false;  // request to raise collector arm (before freight arm raises)
    boolean     needCollectorLowered = false;  // request to lower collector arm (after freight arm lowers)
    boolean     collectorArmRaising  = false;  // commanded to rise, but still in process
    boolean     collectorArmLowering = false;  // commanded to lower, but still in process
    boolean     needEvasiveManeuvers = false;  // One of the buttons pressed will require evasive maneuvers
    boolean     performEvasiveManeuvers = false; // Move the arm to avoid the hub post
    ElapsedTime collectorArmTimer    = new ElapsedTime();

    //freight detection section
    boolean freightDetectionTelemetryEnabled = false;
    boolean ballCubeDetectionTelemetryEnabled = false;
    boolean collectingFreight  = false;
    boolean freightPresent     = false;
    boolean freightIsCube      = false;
    int freightDetectionCounts = 0;
    Gamepad.RumbleEffect ballRumbleEffect1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect ballRumbleEffect2;    // Use to build a custom rumble sequence.

    // These are set in the alliance-specific teleops
    double      duckVelocityNow;
    double      duckVelocityStep;
    TurretPosition      turretAllianceHubAngle;

    double    sonarRangeL=0.0, sonarRangeR=0.0, sonarRangeF=0.0, sonarRangeB=0.0;
    boolean   rangeSensorsEnabled = true;  // enable only when designing an Autonomous plan (takes time!)
    int       rangeSensorIndex = 1;         // only send a new ping out every other control cycle, and rotate sensors
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

        ballRumbleEffect1 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 500 mSec
                .build();
        ballRumbleEffect2 = new Gamepad.RumbleEffect.Builder()
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

            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
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
            processDuckMotorControls();
            processFreightDetector();
            processCollectorArmControl();
            processFreightArmControls();
            processSweeperControls();
            processCappingArmControls();

            // Execute any automatic movements
            robot.cappingArmPosRun();
            robot.freightArmPosRun();

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
            processAutoDriveMode();

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
            telemetry.addData("Duck ", "%.2f (%.0f cts/sec)",  duckVelocityStep,  robot.duckMotorVel );
            telemetry.addData("Sweeper ", "%.2f (%.0f cts/sec)",  robot.sweepMotor.getPower(), robot.sweepMotor.getVelocity() );
            telemetry.addData("Freight Arm", "%d cts %.2f mA", robot.freightMotorPos, robot.freightMotorAmps );
            telemetry.addData("Turret Servo", "%.3f (commanded) %d", robot.turretServo.getPosition(),
                    ((waitForTurretServo)? 1:0) );
            telemetry.addData("Turret Servo Enc", "%d cts stability %d stable %d", robot.turretEncoderPos,
                    robot.turretEncStableCts, ((robot.isTurretStable(robot.turretSetPos))? 1:0) );
            telemetry.addData("Capping Arm", "%d cts %.2f mA", robot.cappingMotorPos, robot.cappingMotorAmps );
            telemetry.addData("Capping Wrist", "%.3f (commanded)", robot.wristServo.getPosition() );
            if( rangeSensorsEnabled ) {
               telemetry.addData("Sonar Range (L/R)", "%.1f  %.1f in", sonarRangeL/2.54, sonarRangeR/2.54 );
               telemetry.addData("Sonar Range (F/B)", "%.1f  %.1f in", sonarRangeF/2.54, sonarRangeB/2.54 );
            }
//          telemetry.addData("Gyro Angle", "%.1f deg (%.1f tilt)", robot.headingIMU(), robot.tiltAngle );
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", elapsedTime, elapsedHz );

            // Testing Color and Distance sensor
            if( freightDetectionTelemetryEnabled ) {
                telemetry.addData("Collector Raised: ", collectorArmRaised);
                telemetry.addData("Collecting Freight: ", collectingFreight);
                telemetry.addData("Freight Distance (mm)", "%.3f", robot.freightDistance);
                telemetry.addData("Freight Present: ", freightPresent);
                telemetry.addData("Freight Is Cube: ", freightIsCube);
            }
            if( ballCubeDetectionTelemetryEnabled ) {
                telemetry.addLine()
                        .addData("Hue", "%.3f", robot.hsvValues[0])
                        .addData("Saturation", "%.3f", robot.hsvValues[1])
                        .addData("Value", "%.3f", robot.hsvValues[2]);
/* COLOR SENSING IS BROKEN (crashes on PLAY)
            telemetry.addData("Alpha", "%.3f", robot.colors.alpha);   // COLORS broken for now
            telemetry.addLine()
                    .addData("Red", "%.3f", robot.colors.red)
                    .addData("Green", "%.3f", robot.colors.green)
                    .addData("Blue", "%.3f", robot.colors.blue);
*/
            }
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//          robot.waitForTick(40);
        } // opModeIsActive

    } // runOpMode

    private void processFreightDetector() {
        if(collectingFreight){
            if (robot.freightPresent()) {
                freightDetectionCounts++;
                // Set freightpresent if set number of detections occurred
                if(freightDetectionCounts > HardwareBothHubs.FREIGHT_DETECTED_THRESHOLD) {
                    freightPresent = true;
                }
            } else {
                freightDetectionCounts = 0;
            }
            if(freightPresent) {
                freightIsCube = robot.freightIsCube();
                if(freightIsCube){
                    gamepad1.rumble(300);
                    gamepad2.rumble(300);
                } else{
                    gamepad1.runRumbleEffect(ballRumbleEffect1);
                    gamepad2.runRumbleEffect(ballRumbleEffect2);
                }
            }
        }
    }

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
        gamepad2_share_last      = gamepad2_share_now;       gamepad2_share_now      = gamepad2.share;
    } // captureGamepad2Buttons

    /*---------------------------------------------------------------------------------*/
    void processSweeperControls() {
        // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button
        if( gamepad2_square_now && !gamepad2_square_last) {
          if( sweeperEjecting ) {  // already reverse; toggle back to forward
//          robot.sweepMotor.setVelocity( sweeperVelocity );  // ON (forward)
            robot.sweepMotor.setPower( sweeperPower );        // ON (forward)
            sweeperRunning  = true;
            sweeperEjecting = false;
          }
          else {  // currently forward, so switch to reverse
//          robot.sweepMotor.setVelocity( -0.5 * sweeperVelocity);  // ON (reverse)
            robot.sweepMotor.setPower( -0.5 * sweeperPower );       // ON (reverse)
            sweeperRunning  = true;
            sweeperEjecting = true;
          }
        } // square
        // Check for an OFF-to-ON toggle of the gamepad1 TRIANGLE button
        if( gamepad2_triangle_now && !gamepad2_triangle_last) {
            robot.sweepMotor.setPower( 0.0 );  // OFF
            sweeperRunning  = false;
            sweeperEjecting = false;
        } // triangle
    } // processSweeperControls

    /*---------------------------------------------------------------------------------*/
    // NOTE: There are two limits that govern our maximum velocity:
    // 1) Motor capability - We can't exceed the velocity associated with 100% power.
    //    For a 435rpm motor,  100% power is 2360 counts/sec (single or dual gecko wheels)
    //    For a 1150rpm motor, 100% power is 2200 counts/sec (wall of wheels)
    // 2) Duck fly-off - Must remain below fly-off velocity until duck reaches the sweeper bar
    // The 1st limit is easily found with setPower(1.0) but the 2nd requires testing.
    void processDuckMotorControls() {
        // Check for an OFF-to-ON toggle of the gamepad1 CIRCLE button
        if( gamepad1_circle_now && !gamepad1_circle_last)
        {   // If running/enabled, turn OFF and reset
            if( duckMotorEnable ) {
                robot.duckMotor.setVelocity( 0.0 );
                duckVelocityNow = duckVelocityStep;  // back to starting velocity;
                duckMotorEnable = false;
                robot.writeDuckLog();
            }
            // If stopped/disabled, turn ON at our initial velocity
            else {
//              robot.duckMotor.setPower( 1.0 );  // to determine max counts/sec
                robot.duckMotor.setVelocity( duckVelocityNow );
                duckMotorEnable = true;
                // If logging instrumentation, begin a new dataset now:
                if( robot.duckMotorLogging ) {
                    robot.duckMotorLogIndex  = 0;     // reset for another log
                    robot.duckMotorLogEnable = true;  // enable logging
                    robot.duckMotorTimer.reset();     // reset timestamping
                }
            }
        }
        // No operator change (either start/stop), but what about ramping?
        else if( duckMotorEnable ) {
            // Have we already ramped up to the maximum velocity?
            if( Math.abs(duckVelocityNow) < 2300 ) {  // allows 2300 cps max
                duckVelocityNow += duckVelocityStep;
                robot.duckMotor.setVelocity( duckVelocityNow );
            }
        }
    } // processDuckMotorControls

    /*---------------------------------------------------------------------------------*/
    double determineBoxServoDumpAngle() {
        double servoTarget   = robot.BOX_SERVO_DUMP_BOTTOM; // updated below...
        int    freightArmPos = robot.freightMotorPos;       // current encoder count
        int    midpoint1     = (robot.FREIGHT_ARM_POS_HUB_TOP    + robot.FREIGHT_ARM_POS_HUB_MIDDLE)/2;
        int    midpoint2     = (robot.FREIGHT_ARM_POS_HUB_MIDDLE + robot.FREIGHT_ARM_POS_HUB_BOTTOM)/2;
        // Determine servo DUMP ANGLE based on freight-arm location
        if( freightArmPos < robot.FREIGHT_ARM_POS_VERTICAL )
            servoTarget = robot.BOX_SERVO_DUMP_FRONT;
        else if( freightArmPos < midpoint1 )
            servoTarget = robot.BOX_SERVO_DUMP_TOP;
        else if( freightArmPos < midpoint2 )
            servoTarget = robot.BOX_SERVO_DUMP_MIDDLE;
        else 
            servoTarget = robot.BOX_SERVO_DUMP_BOTTOM;
        
        return servoTarget;
    } // determineBoxServoDumpAngle

    /*---------------------------------------------------------------------------------*/
    // The only time the collector arm should be LOWERED is when we're collecting.
    // All other times it can/should be RAISED so the freight arm is free to move.
    void processCollectorArmControl() {
        
        // Can we clear any requests? (we're in the desired state)
        if( needCollectorLowered && (collectorArmRaised == false) ) {
            // Already there; clear the request
            needCollectorLowered = false;
        }
        if( needCollectorRaised && (collectorArmRaised == true) ) {
            // Already there; clear the request
            needCollectorRaised = false;
        }

        // We only raise/lower the collector-arm when the freight-arm is in the correct position
        boolean safeToMoveCollectorArm = (robot.freightMotorPos < robot.FREIGHT_ARM_POS_SAFE)? true:false;
        
        // Process request to lower (if not already underway)
        if( safeToMoveCollectorArm && needCollectorLowered && !collectorArmLowering ) {
             robot.linkServo.setPosition( robot.LINK_SERVO_LOWERED );
             collectorArmTimer.reset(); // start our timer
             collectorArmLowering = true;
             // automatically turn ON the sweeper as the arm is lowered
//           robot.sweepMotor.setVelocity( sweeperVelocity );  // ON (forward)
             robot.sweepMotor.setPower( sweeperPower );        // ON (forward)
             sweeperRunning = true;
        } // lower

        // Process request to raise (if not already underway)
        if( safeToMoveCollectorArm && needCollectorRaised && !collectorArmRaising ) {
             robot.linkServo.setPosition( robot.LINK_SERVO_RAISED );
             collectorArmTimer.reset(); // start our timer
             collectorArmRaising = true;
        } // raise

        // Have we finished lowering (takes 300 msec)?
        if( collectorArmLowering && (collectorArmTimer.milliseconds() >= 350) ) {
            collectorArmLowering = false;  // back to idle
            collectorArmRaised   = false;  // LOWERED!
        }

        // Have we finished raising (takes 500 msec)?
        if( collectorArmRaising && (collectorArmTimer.milliseconds() >= 550) ) {
            collectorArmRaising = false;  // back to idle
            collectorArmRaised  = true;   // RAISED!
        }
     
    } // processCollectorArmControl

    /*---------------------------------------------------------------------------------
    /* The angle we can safely swing LEFT or RIGHT from center varies by arm elevation: 
    /*                      0.500
    /*                    +--+--+ COLLECT LEVEL 
    /*                    |  |  |
    /*                    |  |  | TURRET_RIGHT_WHEEL
    /*                    |  |  +--------------------+ 0.005
    /*    COLLECTOR ARM   |  |                       |
    /*    0.740 +---------+  |                       |
    /*          |            |                       |
    /*          |            |   <<VERTICAL>>        |
    /*          |            |                       |
    /*          |            |                       | TURRET_RIGHT_MAX
    /*          |            |         +-------------+
    /*          |            |         |      COLLECTOR ARM POST
    /*          |            |         |
    /*          +------------+---------+
    /*---------------------------------------------------------------------------------*/
    boolean safeToSwingRight() {
        double currentElev   = robot.freightMotorPos;
        double currentAngle  = robot.turretTargetPos;
        boolean frontControl   = (robot.freightMotorPos < robot.FREIGHT_ARM_POS_VERTICAL);
        double turretDecrement = (frontControl)? turretStepSize : (3.0 * turretStepSize);
        double proposedAngle = currentAngle - turretDecrement; // RIGHT
        boolean safeToStepRight;
        // Determine if motion to the RIGHT is safe for this arm elevation
        if( currentElev < robot.TURRET_RIGHT_WHEEL ) {
            safeToStepRight = (proposedAngle > 0.500);  // back to centerline
        }
        else if( currentElev < robot.FREIGHT_ARM_POS_VERTICAL ){
            safeToStepRight = (proposedAngle > 0.005);
        }
        else if( currentElev < robot.TURRET_RIGHT_MAX ) {
            safeToStepRight = (proposedAngle > 0.005);  // ???
        }
        else { // rotated back down toward the floor on the other side
            safeToStepRight = (proposedAngle > 0.500);  // back to centerline
        }
        return safeToStepRight;
    } // safeToSwingRight

    /*---------------------------------------------------------------------------------*/
    boolean safeToSwingLeft() {
        double currentElev   = robot.freightMotorPos;
        double currentAngle  = robot.turretTargetPos;
        boolean frontControl   = (robot.freightMotorPos < robot.FREIGHT_ARM_POS_VERTICAL);
        double turretIncrement = (frontControl)? turretStepSize : (3.0 * turretStepSize);
        double proposedAngle = currentAngle + turretIncrement;  // LEFT
        boolean safeToStepLeft;
        // Determine if motion to the LEFT is safe for this arm elevation
        if( currentElev < robot.TURRET_LEFT_COLLECTOR1 ) {
            safeToStepLeft = (proposedAngle < 0.500);  // back to centerline
        }
        else if( currentElev < robot.TURRET_LEFT_COLLECTOR2 ) {
            safeToStepLeft = (proposedAngle < 0.740);
        }
        else if( currentElev < robot.FREIGHT_ARM_POS_VERTICAL ) {
            safeToStepLeft = (proposedAngle < 0.740);  // ???
        }
        else if( currentElev < robot.TURRET_LEFT_MAX ) {
            safeToStepLeft = (proposedAngle < 1.000);
        }
        else {
            safeToStepLeft = (proposedAngle < 0.500);  // back to centerline
        }
        return safeToStepLeft;
    } // safeToSwingLeft

    /*---------------------------------------------------------------------------------*/
    void processFreightArmControls() {
        boolean safeToManuallyLower  = collectorArmRaised && (robot.freightMotorPos > robot.FREIGHT_ARM_POS_SPIN);
        boolean safeToManuallyRaise  = collectorArmRaised && (robot.freightMotorPos < robot.FREIGHT_ARM_POS_MAX);
        // We want manual "raise" and "lower" to work correctly on both sides of vertical
        boolean frontControl = (robot.freightMotorPos < robot.FREIGHT_ARM_POS_VERTICAL);
        // For both FRONT (shared hub) & REAR (alliance hub), pushing forward/back should RAISE/LOWER arm
        // Before any modification, pushing forward on left_stick_y produces NEGATIVE values.
        double  gamepad2_left_stick = gamepad2.left_stick_y;
        boolean manual_elev_control = ( Math.abs(gamepad2_left_stick) > 0.05 );
        // Pushing the right stick left/right swings turret counter-clockwise or clockwise
        double  gamepad2_right_stick  = gamepad2.right_stick_x;
        boolean manual_turret_control = ( Math.abs(gamepad2_right_stick) > 0.05 );

        // Check for an OFF-to-ON toggle of the gamepad2 RIGHT BUMPER
        if( gamepad2_r_bumper_now && !gamepad2_r_bumper_last)
        {   // Ignore requests to dump if still in the COLLECT position
            if( robot.freightMotorPos > robot.FREIGHT_ARM_POS_SPIN ) {
              double boxServoTarget = determineBoxServoDumpAngle();
              robot.boxServo.setPosition( boxServoTarget );   // DUMP!
            }
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 SHARE button
        if( gamepad2_share_now && !gamepad2_share_last )
        {
            robot.turretPositionSet( TurretPosition.CENTERED );
            robot.freightArmPosInit( robot.FREIGHT_ARM_POS_COLLECT );
            freightArmTarget  = robot.FREIGHT_ARM_POS_COLLECT;
            robot.boxServo.setPosition( robot.BOX_SERVO_COLLECT );
            robot.sweepMotor.setPower( 0.0 );  // OFF
            sweeperRunning    = false;
            collectingFreight = false;
            freightArmCycleCount = FREIGHT_CYCLECOUNT_DONE;
            // Also reset collector arm to raised position
            robot.linkServo.setPosition( robot.LINK_SERVO_RAISED );
            collectorArmRaised   = true;   // RAISED!
            collectorArmRaising  = false;  // back to idle
            collectorArmLowering = false;
            needCollectorRaised  = false;
            needCollectorLowered = false;
        }

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 CIRCLE button
        if(( gamepad2_circle_now && !gamepad2_circle_last) || (freightPresent && collectingFreight))
        {
            // Do we need to re-center turret before we can begin to raise/lower freight-arm?
            if( needEvasiveManeuvers ) {
//              robot.turretPositionSet(TurretPosition.CENTERED);
                performEvasiveManeuvers = true;
                needEvasiveManeuvers = false;
            } else {
                robot.turretPositionSet(TurretPosition.CENTERED);
            }
            needCollectorRaised  = true;
            freightArmTarget     = robot.FREIGHT_ARM_POS_TRANSPORT1;
            freightArmServoPos   = robot.BOX_SERVO_TRANSPORT;
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
            // rotate box so freight doesn't fall out (but dumps any DOUBLE-FREIGHT!)
            robot.boxServo.setPosition( robot.BOX_SERVO_STORED );
            // automatically turn OFF the sweeper
            robot.sweepMotor.setPower( 0.0 );  // OFF
            sweeperRunning    = false;
            collectingFreight = false;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 CROSS button ()
        else if( gamepad2_cross_now && !gamepad2_cross_last)
        {
            // Do we need to re-center turret before we can begin to raise/lower freight-arm?
            robot.turretPositionSet(TurretPosition.CENTERED);
            // We lower from front-dump really fast.  Get the servo moving
            // so the front edge of box isn't pointing down!
            robot.boxServo.setPosition( robot.BOX_SERVO_COLLECT );
            needCollectorLowered = true; // at the END! (AFTER freight-arm is in place)
            if(robot.freightMotorPos < 500) {
                waitForDumpServo = true;
                // Hold for 500ms if we are going front dump to collect
                freightArmDelayTimer.reset();
            }
            freightArmTarget     = robot.FREIGHT_ARM_POS_COLLECT;
            freightArmServoPos   = robot.BOX_SERVO_COLLECT;
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }

        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD UP
        else if( gamepad2_dpad_up_now && !gamepad2_dpad_up_last)
        {
            // Do we need to re-center turret before we can begin to raise/lower freight-arm?
            robot.turretPositionSet(TurretPosition.CENTERED);
            // If turret manually rotated, don't snap back to center until above post height
            needEvasiveManeuvers = true;
            needCollectorRaised  = true;
            freightArmTarget     = robot.FREIGHT_ARM_POS_HUB_TOP;
            freightArmServoPos   = robot.BOX_SERVO_TRANSPORT;
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD LEFT
        else if( gamepad2_dpad_left_now && !gamepad2_dpad_left_last)
        {
            // Do we need to re-center turret before we can begin to raise/lower freight-arm?
            robot.turretPositionSet(TurretPosition.CENTERED);
            needEvasiveManeuvers = true;
            autoRotateTurret     = true;
            needCollectorRaised  = true;
            freightArmTarget     = robot.FREIGHT_ARM_POS_HUB_TOP;  // TOP + ROTATE!
            freightArmServoPos   = robot.BOX_SERVO_TRANSPORT;
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD RIGHT
        else if( gamepad2_dpad_right_now && !gamepad2_dpad_right_last)
        {
            // Do we need to re-center turret before we can begin to raise/lower freight-arm?
            robot.turretPositionSet(TurretPosition.CENTERED);
            needCollectorRaised  = true;
            freightArmTarget     = robot.FREIGHT_ARM_POS_SHARED;
            freightArmServoPos   = robot.BOX_SERVO_TRANSPORT;
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD DOWN
        else if( gamepad2_dpad_down_now && !gamepad2_dpad_down_last)
        {
            // Do we need to re-center turret before we can begin to raise/lower freight-arm?
            robot.turretPositionSet(TurretPosition.CENTERED);
            needCollectorRaised  = true;
            freightArmTarget     = robot.FREIGHT_ARM_POS_HUB_BOTTOM;
            freightArmServoPos   = robot.BOX_SERVO_TRANSPORT;
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        else if( manual_elev_control || freightArmElevTweaked ) {
            // Abort any automatic movement in progress (we don't have to zero
            // any motor power in progress because we're about to set a new value)
            robot.freightMotorAuto = false;
            freightArmCycleCount = FREIGHT_CYCLECOUNT_DONE;
            // Different amounts of motor power are needed to raise and lower the arm
            // and EVEN MORE power may be needed if lifting a double-weighted cube
            double frontRaisePwr = 0.40 + (0.40 * -gamepad2_left_stick); // from 0.40 to 0.80
            double rearRaisePwr  = 0.40 + (0.25 * -gamepad2_left_stick); // from 0.40 to 0.65
            double frontLowerPwr = 0.20 + (0.10 *  gamepad2_left_stick); // from 0.20 to 0.30
            double rearLowerPwr  = 0.20 + (0.10 *  gamepad2_left_stick); // from 0.20 to 0.30
            // Does user want to raise the arm (negative joystick input)
            if( safeToManuallyRaise && (gamepad2_left_stick < -0.05) ) {
                double motorPower = (frontControl)? frontRaisePwr : -rearRaisePwr;
                robot.freightMotor.setPower( motorPower );
                freightArmElevTweaked = true;
            }
            else if( safeToManuallyLower && (gamepad2_left_stick > 0.05) ) {
                double motorPower = (frontControl)? -frontLowerPwr : rearLowerPwr;
                robot.freightMotor.setPower( motorPower );
                freightArmElevTweaked = true;
            }
            else if( freightArmElevTweaked ) {
                robot.freightMotor.setPower( 0.0 );
                freightArmElevTweaked = false;
            }
        } // manual_elev_control
        else if( manual_turret_control && (robot.freightMotorAuto == false) ) {
            freightArmCycleCount = FREIGHT_CYCLECOUNT_DONE;
            // Does user want to swing right?
            if( safeToSwingRight() && (gamepad2_right_stick > 0.30) ) {
                robot.turretPositionShift( -turretStepSize );
            }
            else if( safeToSwingLeft() && (gamepad2_right_stick < -0.30) ) {
                robot.turretPositionShift( turretStepSize );
            }
        } // manual_turret_control

        //===================================================================
        if( freightArmCycleCount >= FREIGHT_CYCLECOUNT_START ) {
            // Collector arm must be raised before any freight arm motion is commanded
            if( collectorArmRaised ) {
                // Turret must be centered before motion starts (except for evasive manuevers)
                if( !performEvasiveManeuvers && !robot.isTurretStable( TurretPosition.CENTERED ) ) {
                    // Do nothing (wait for servo to rotate back to center
                }
                else if(waitForDumpServo) {
                    if(freightArmDelayTimer.milliseconds() >= 300) {
                        waitForDumpServo = false;
                    }
                    else {
                        // Do nothing (wait for servo to rotate back upward
                        // before lowering the arm to the field floor.
                    }
                } else {
                    robot.freightArmPosInit( freightArmTarget );
                    freightArmCycleCount--;      // exit this state
                }
            }
            else {
               // wait for collector arm!
            }
        }
        else if( freightArmCycleCount == FREIGHT_ARM_START_ROTATING_TURRET) {
            if (robot.freightMotorAuto ) {
                if( performEvasiveManeuvers) {
                    if( robot.freightMotorPos <= robot.FREIGHT_ARM_POS_EVASIVE ) {
                        robot.turretPositionSet(TurretPosition.CENTERED);
                        freightArmCycleCount--;
                    }
                } else if( autoRotateTurret ) {
                    if( robot.freightMotorPos >= robot.FREIGHT_ARM_POS_ROT_TURRET ) {
                        robot.turretPositionSet(turretAllianceHubAngle);
                        autoRotateTurret = false;
                    }
                } else {
                    freightArmCycleCount--;
                }
            } else {
                freightArmCycleCount--;
            }
        }
        else if( freightArmCycleCount > FREIGHT_CYCLECOUNT_SERVO ) {
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
            // If we're still moving then HOLD at this freightArmCycleCount value
            if( robot.freightMotorAuto ) {
                // While waiting, see if we're clear to start rotating 
                // freight arm turret around toward the alliance hub?
                if( performEvasiveManeuvers ) {
                    freightArmCycleCount = FREIGHT_ARM_CENTERING;
                    performEvasiveManeuvers = false;
                }
            }
            else { // arm motor is stopped, but arm may be bouncing
                freightArmCycleCount = FREIGHT_CYCLECOUNT_SETTLE;
                freightArmDelayTimer.reset();
            }
        }
        else if( freightArmCycleCount == FREIGHT_ARM_CENTERING ) {
            if(!robot.isTurretStable( TurretPosition.CENTERED )) {
                // We have gone too far without being centered, so now we need to wait for
                // the servo.
                if( robot.freightMotorPos <= robot.FREIGHT_ARM_POS_ROT_TURRET ) {
                    robot.freightMotorAuto = false;
                    robot.freightMotor.setPower( 0.0 );
                    waitingForTurret = true;
                }
            } else {
                if( waitingForTurret ) {
                    waitingForTurret = false;
                    // Start the arm moving again
                    robot.freightArmPosInit( freightArmTarget );
                } else {
                    // arm motor is stopped, but arm may be bouncing
                    if( !robot.freightMotorAuto ) {
                        freightArmCycleCount = FREIGHT_CYCLECOUNT_SETTLE;
                        freightArmDelayTimer.reset();
                    }
                }
            }
        }
        else if( freightArmCycleCount == FREIGHT_CYCLECOUNT_SETTLE) {
            if( freightArmDelayTimer.milliseconds() > 500 ) {
                freightArmCycleCount--;
                // If we stopped in COLLECT position then sweeper will be
                // running so use that to restart our freight detection flags
                if( sweeperRunning && !collectingFreight ) {
                    freightPresent = false;
                    freightIsCube = false;
                    collectingFreight = true;
                }
            }
        }
    } // processFreightArmControls

    /*---------------------------------------------------------------------------------*/
    /* The 1150rpm capping-arm motor requires a minimum of 20% power to RAISE the arm  */
    /* and 10% power to LOWER the arm -- where RAISE and LOWER switch orientation as   */
    /* the arm rotates thru the full range of encoder counts. This function scales the */
    /* manual user input (left/right trigger) to allow fine control over the full      */
    /* range of motion.                                                                */
    double determineCappingMotorPower( double percentInput ) {
        int countsFromVertical = robot.cappingMotorPos - robot.CAPPING_ARM_POS_VERTICAL;
        double absPercentInput = Math.abs( percentInput );
        // The necessary min/max power range is determined by these inputs:
        // a) percentInput > 0 = rotation toward CAPPING_ARM_POS_GRAB (gamepad1.left_trigger) 
        //      countsFromVertical >= 0 = LOWERING the capping arm (need 10%+ power)
        //      countsFromVertical <  0 = RAISING the capping arm  (need 20%+ power)
        // b) percentInput < 0 = rotation toward CAPPING_ARM_POS_START (gamepad1.right_trigger) 
        //      countsFromVertical >= 0 = RAISING the capping arm  (need 20%+ power)
        //      countsFromVertical <  0 = LOWERING the capping arm (need 10%+ power)
        boolean raisingArm = ((percentInput > 0.0) && (countsFromVertical < 0)) ||
                             ((percentInput < 0.0) && (countsFromVertical >= 0));
        double minPower = (raisingArm)? 0.20 : 0.10;
        // scale trigger input to 1/4, unless trying to go fast and then use 1/2
        double scaleFactor = (absPercentInput < 0.80)? 0.25 : 0.50;
        // maxPower = minPower + 0.50 (0.60 for lowering; 0.70 for raising)
        double computedPower = minPower + (scaleFactor * absPercentInput);
        // Apply the positive/negative sign from percentInput
        computedPower *= (percentInput > 0.0)? 1.0 : -1.0;
//      telemetry.addData("CappingMotor", "%.2f", computedPower );
        return computedPower;
    } // determineCappingMotorPower

    /*---------------------------------------------------------------------------------*/
    void processCappingArmControls() {
        double gamepad1_left_trigger  = gamepad1.left_trigger;
        double gamepad1_right_trigger = gamepad1.right_trigger;
        boolean manual_trigger_control = ((gamepad1_left_trigger  > 0.03) ||
                                          (gamepad1_right_trigger > 0.03));
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
            // <MIN> STORE ... LIBERTY ... VERTICAL ... CAP ... GRAB <MAX>
            int midpoint1 = (robot.CAPPING_ARM_POS_STORE + robot.CAPPING_ARM_POS_LIBERTY)/2;
            int midpoint2 = (robot.CAPPING_ARM_POS_CAP   + robot.CAPPING_ARM_POS_GRAB)/2;
            // toggle into and out of CAP position (use current arm position to decide)
            if( (robot.cappingMotorPos < midpoint1) ||   /* currently STORE */
                (robot.cappingMotorPos > midpoint2) )    /* currently GRAB  */
            {  // switch to CAP
            wristServoPos =  robot.WRIST_SERVO_CAP;
            robot.cappingArmPosInit( robot.CAPPING_ARM_POS_CAP );
            cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
            else
            { // currently CAP; switch to STORE
              wristServoPos = robot.WRIST_SERVO_STORE;
              robot.cappingArmPosInit( robot.CAPPING_ARM_POS_STORE );
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
            {  // currently STORE; switch to GRAB/CUP
//            wristServoPos = robot.WRIST_SERVO_GRAB;
              wristServoPos = robot.WRIST_SERVO_CUP;
//            robot.cappingArmPosInit( robot.CAPPING_ARM_POS_GRAB );
              robot.cappingArmPosInit( robot.CAPPING_ARM_POS_CUP );
              cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
            else
            { // currently GRAB; switch to STORE
              wristServoPos = robot.WRIST_SERVO_STORE;
              robot.cappingArmPosInit( robot.CAPPING_ARM_POS_STORE );
              cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
            }
        }
        else if( manual_trigger_control || cappingArmTweaked ) {
            // Abort any automatic movement in progress
            robot.cappingMotorAuto = false;
            cappingArmCycleCount = CAPPING_CYCLECOUNT_DONE;
            // Does user want to lower
            if( gamepad1_left_trigger > 0.03 ) {
                // limit how far we can drive this direction
                if( robot.cappingMotorPos < robot.CAPPING_ARM_POS_GRAB ) {
                    double motorPower = determineCappingMotorPower( +gamepad1_left_trigger );
                    robot.cappingMotor.setPower( motorPower );
                    cappingArmTweaked = true;
                }
                else {  // stop at the GRAB position
                    robot.cappingMotor.setPower( 0.0 );
                }
            }
            else if( gamepad1_right_trigger > 0.03 ) {
                // limit how far we can drive this direction
                if( robot.cappingMotorPos > 0 ) {
                    double motorPower = determineCappingMotorPower( -gamepad1_right_trigger );
                    robot.cappingMotor.setPower( motorPower );
                    cappingArmTweaked = true;
                }
                else { // stop at the STORED position
                    robot.cappingMotor.setPower( 0.0 );
                }
            }
            else if( cappingArmTweaked ) {
                robot.cappingMotor.setPower( 0.0 );
                cappingArmTweaked = false;
            }
        } // manual control

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
            if( robot.cappingMotorAuto ) {
                // still moving; hold at this cycle count
            }
            else { // no longer busy; transition to DONE state
                cappingArmCycleCount = CAPPING_CYCLECOUNT_DONE;
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
    void processAutoDriveMode() {
        if( autoDrive ) {
            // Update our tilt angle information
            robot.headingIMU();

            // Do we need to break from autoDrive due to user input?
            if( breakFromAutoDrive() ) {
                robot.stopMotion();
                autoDrive = false;
            }
            // Do we need to break from autoDrive because we've reached the goal
            else if(robot.tiltAngle < HardwareBothHubs.BARRIER_NESTED_ROBOT_TILT_TELE) {
                robot.stopMotion();
                autoDrive = false;
            }
        } // autoDrive
    } // processAutoDriveMode

    /*---------------------------------------------------------------------------------*/
    boolean breakFromAutoDrive() {
        boolean breakAutoDrive =
                        gamepad1.dpad_down || gamepad1.dpad_up    ||
                        gamepad1.dpad_left || gamepad1.dpad_right ||
                        (Math.abs(gamepad1.left_stick_x)  > 0.02) ||
                        (Math.abs(gamepad1.left_stick_y)  > 0.02) ||
                        (Math.abs(gamepad1.right_stick_x) > 0.02) ||
                        (Math.abs(gamepad1.right_stick_y) > 0.02);
        return breakAutoDrive;
    } // breakFromAutoDrive

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
//          yTranslation = multSegLinearXY( -gamepad1.left_stick_y );
//          xTranslation = multSegLinearXY(  gamepad1.left_stick_x );
//          rotation     = multSegLinearRot( -gamepad1.right_stick_x );
            yTranslation = -gamepad1.left_stick_y * 1.00;
            xTranslation =  gamepad1.left_stick_x * 1.25;
            rotation     = -gamepad1.right_stick_x * 0.50;
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
    void processRangeSensors( int sensorNum ) {
        // only send one ping per control cycle (left, right, front, or back)
        switch( sensorNum ) {
            case 1 : sonarRangeL = robot.updateSonarRangeL(); break;
            case 3 : sonarRangeR = robot.updateSonarRangeR(); break;
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
          sonarRangeR = robot.updateSonarRangeR();
          sonarRangeF = robot.updateSonarRangeF();
          sonarRangeB = robot.updateSonarRangeB();
        }
    } // averagedRangeSensors

} // Teleop
