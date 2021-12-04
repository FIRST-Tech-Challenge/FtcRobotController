/* FTC Team 7572 - Version 1.2 (12/03/2021)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp Full Control.
 */
@TeleOp(name="Teleop-Red", group="7592")
//@Disabled
public class TeleopRed extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;  // Duck motor on/off
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;  // Backwards Drive mode (also turns off driver-centric mode)
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;  // sweeper on/off (collect)
    boolean gamepad1_square_last,     gamepad1_square_now     = false;  // Enables/calibrates driver-centric mode
//  boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;  // gamepad1.dpad_up used live/realtime
//  boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;  //   (see processDpadDriveMode() below)
//  boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
//  boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;  // Duck Motor (slower)
    boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;  // Duck Motor (faster)

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  // Capping Arm (Capping Position)
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  // Freight Arm (Transport Horizontal)
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  // Freight Arm (Collect)
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  // claw servo open/close 
    boolean gamepad2_dpad_up_last,    gamepad2_dpad_up_now    = false;  // Freight Arm (Hub-Top)
    boolean gamepad2_dpad_down_last,  gamepad2_dpad_down_now  = false;  // Freight Arm (Hub-Bottom) 
    boolean gamepad2_dpad_left_last,  gamepad2_dpad_left_now  = false;  // Freight Arm (Raise/Spin)
    boolean gamepad2_dpad_right_last, gamepad2_dpad_right_now = false;  // Freight Arm (Hub-Middle)
    boolean gamepad2_l_bumper_last,   gamepad2_l_bumper_now   = false;  // sweeper (reverse)
    boolean gamepad2_r_bumper_last,   gamepad2_r_bumper_now   = false;  // box servo (dump)

    boolean sweeperRunning  = false;  // controlled by driver (gamepad1)
    boolean sweeperReversed = false;  // controlled by operator (gamepad2)

    boolean clawServoOpen   = false;  // true=OPEN; false=CLOSED on team element

    double    freightArmServoPos = 0.48;      // Which servo setting to target once movement starts

    final int FREIGHT_CYCLECOUNT_START = 20;  // Freight Arm just started moving (1st cycle)
    final int FREIGHT_CYCLECOUNT_SERVO = 10;  // Freight Arm off the floor (safe to rotate box servo)
    final int FREIGHT_CYCLECOUNT_CHECK = 1;   // Time to check if Freight Arm is still moving?
    final int FREIGHT_CYCLECOUNT_DONE  = 0;   // Movement is complete (cycle count is reset)
    int       freightArmCycleCount     = FREIGHT_CYCLECOUNT_DONE;
    boolean   freightArmTweaked        = false;  // Reminder to zero power when trigger released

    double    wristServoPos = 0.950;          // Which servo setting to target once capping arm movement starts

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
    double    duckPower = 0.670;  //red (positive!)
    double    duckVelocity = 1600;      // target counts per second

    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    elapsedTime, elapsedHz;

    /* Declare OpMode members. */
    HardwareBothHubs robot = new HardwareBothHubs();
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap);

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

            // Process all the driver/operator inputs
            processSweeperControls();
            processDuckMotorControls();
            processFreightArmControls();
            processCappingArmControls();

            // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button (toggles DRIVER-CENTRIC drive control)
            if( gamepad1_square_now && !gamepad1_square_last)
            {
                driverMode = DRIVER_MODE_DRV_CENTRIC;
            }

            // Check for an OFF-to-ON toggle of the gamepad1 TRIANGLE button (toggles STANDARD/BACKWARD drive control)
            if( gamepad1_triangle_now && !gamepad1_triangle_last)
            {
                // If currently in DRIVER-CENTRIC mode, switch to STANDARD (robot-centric) mode
                if( driverMode != DRIVER_MODE_STANDARD ) {
                    driverMode = DRIVER_MODE_STANDARD;
//                  backwardDriveControl = false;  // reset to forward mode
                }
                // Already in STANDARD mode; Just toggle forward/backward mode
                else {  //(disabled for now)
//                  backwardDriveControl = !backwardDriveControl; // reverses which end of robot is "FRONT"
                }
            }

            if( false ) {
                telemetry.addData("triangle", "Robot-centric");
                telemetry.addData("square", "Driver-centric (set joystick!)");
                telemetry.addData("d-pad", "Fine control (20%)");
                telemetry.addData("circle", "Duck Motor On/Off");
                telemetry.addData("bumpers", "Duck Motor Speed");
                telemetry.addData("bumpers", "Duck Motor Speed");
                telemetry.addData("cross", "Sweeper On/Off");
                telemetry.addData(" ", " ");
            }

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
            telemetry.addData("Front", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
                    frontLeft, robot.frontLeftMotorVel, frontRight, robot.frontRightMotorVel );
            telemetry.addData("Back ", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
                    rearLeft,  robot.rearLeftMotorVel,  rearRight,  robot.rearRightMotorVel );
            telemetry.addData("Duck ", "%.2f (%.0f cts/sec)",
                    duckPower,  robot.duckMotorVel );
            telemetry.addData("Freight Arm", "%d cts %.2f mA", robot.freightMotorPos, robot.freightMotorAmps );
            telemetry.addData("Capping Arm", "%d cts %.2f mA", robot.cappingMotorPos, robot.cappingMotorAmps );
            telemetry.addData("Capping Wrist", "%.3f (commanded)", robot.wristServo.getPosition() );
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
        gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
        gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
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
            robot.sweepServo.setPower( -0.25 );  // ON (reverse)
            sweeperReversed = true;   // note that we need to turn it back OFF
        }
        // or not. but was PREVIOUSLY
        else if( sweeperReversed ) {
            robot.sweepServo.setPower( 0.0 );  // OFF
            sweeperReversed = false;   // only do this once!
        }
        // Check for an OFF-to-ON toggle of the gamepad1 CROSS button
        else if( gamepad1_cross_now && !gamepad1_cross_last) {
            if( sweeperRunning ) {  // currently running, so toggle OFF
                robot.sweepServo.setPower( 0.0 );  // OFF
            }
            else {  // currently stopped, so toggle ON
                robot.sweepServo.setPower( 1.0 );  // ON (forward)
            }
            sweeperRunning = !sweeperRunning;
        }
    } // processSweeperControls

    /*---------------------------------------------------------------------------------*/
    void processDuckMotorControls() {
        // Check for an OFF-to-ON toggle of the gamepad1 CIRCLE button
        if( gamepad1_circle_now && !gamepad1_circle_last)
        {   // toggle motor ON/OFF
            if( duckMotorEnable ) {
                robot.duckMotor.setPower( 0.0 );
            }
            else {
                robot.duckMotor.setPower( duckPower );
//              robot.duckMotor.setVelocity( duckVelocity );
            }
            duckMotorEnable = !duckMotorEnable;
        }
        // Check for an OFF-to-ON toggle of the gamepad1 left bumper
        if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last)
        {
            // decrease duck motor speed
            duckPower -= 0.02;   // 2% decrements
//          duckVelocity -= 20;
            if( duckMotorEnable ) {
                robot.duckMotor.setPower( duckPower );
//              robot.duckMotor.setVelocity( duckVelocity );
            }
        }
        // Check for an OFF-to-ON toggle of the gamepad1 right bumper
        if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last)
        {
            // increase duck motor speed
            duckPower += 0.02;   // 2% increments
//          duckVelocity += 20;
            if( duckMotorEnable ) {
                robot.duckMotor.setPower( duckPower );
//              robot.duckMotor.setVelocity( duckVelocity );
            }
        }
    } // processDuckMotorControls

    /*---------------------------------------------------------------------------------*/
    void processFreightArmControls() {
        // Check for an OFF-to-ON toggle of the gamepad2 RIGHT BUMPER
        if( gamepad2_r_bumper_now && !gamepad2_r_bumper_last)
        {
            robot.boxServo.setPosition( robot.BOX_SERVO_DUMP_TOP );
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
            // automatically turn on the sweeper
            robot.sweepServo.setPower( 1.0 );  // ON (forward)
            sweeperRunning = true;
        }
        //===================================================================
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD UP
        if( gamepad2_dpad_up_now && !gamepad2_dpad_up_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_TRANSPORT;
//          robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_TOP, 0.80 );
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_TOP, 1.0 );
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD LEFT
        if( gamepad2_dpad_left_now && !gamepad2_dpad_left_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_TRANSPORT;
//          robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_MIDDLE, 0.50 );
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_MIDDLE, 1.0 );
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD RIGHT
        if( gamepad2_dpad_right_now && !gamepad2_dpad_right_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_TRANSPORT;
//          robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_MIDDLE, 0.50 );
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_MIDDLE, 1.0 );
            freightArmCycleCount = FREIGHT_CYCLECOUNT_START;
        }
        // Check for an OFF-to-ON toggle of the gamepad2 DPAD DOWN
        if( gamepad2_dpad_down_now && !gamepad2_dpad_down_last)
        {
            freightArmServoPos =  robot.BOX_SERVO_TRANSPORT;
//          robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_BOTTOM, 0.30 );
            robot.freightArmPosition( robot.FREIGHT_ARM_POS_HUB_BOTTOM, 1.0 );
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
            if( gamepad2.left_trigger > 0.05 ) {
                robot.freightMotor.setPower( 0.10 );
                freightArmTweaked = true;
            }
            else if( gamepad2.right_trigger > 0.05 ) {
                robot.freightMotor.setPower( -0.10 );
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
        // Check for an OFF-to-ON toggle of the gamepad2 TRIANGLE button
        if( gamepad2_triangle_now && !gamepad2_triangle_last)
        {
            wristServoPos =  robot.WRIST_SERVO_CAP;
            robot.cappingArmPosition( robot.CAPPING_ARM_POS_CAP, 0.70 );
            cappingArmCycleCount = CAPPING_CYCLECOUNT_START;
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
            double gamepad2_left_stick_y = gamepad2.left_stick_y;
            double cappingMotorPower = 0.25 * gamepad2_left_stick_y;
            if( gamepad2_left_stick_y < -0.05 ) {
                // limit how far we can drive this direction
                if( robot.cappingMotorPos > 0 ) {
                    robot.cappingMotor.setPower( cappingMotorPower );
                    cappingArmTweaked = true;
                }
                else {
                    robot.cappingMotor.setPower( 0.0 );
                }
            }
            else if( gamepad2_left_stick_y > 0.05 ) {
                // limit how far we can drive this direction
                if( robot.cappingMotorPos < robot.CAPPING_ARM_POS_GRAB ) {
                    robot.cappingMotor.setPower( cappingMotorPower );
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
        // Check for an OFF-to-ON toggle of the gamepad2 SQUARE button
        if( gamepad2_square_now && !gamepad2_square_last)
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
        if( gamepad2.right_stick_y < -0.03 ) {
            // What was the last commanded position?
            double curPos = robot.wristServo.getPosition();
            if( curPos >  -0.95 ) {
                double newPos = curPos - 0.005;
                robot.wristServo.setPosition( newPos );
            }
        }
        else if( gamepad2.right_stick_y > 0.03 ) {
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
        double fineTurnSpeed = 0.05;
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
        else if(gamepad1.left_trigger>0.10){
            telemetry.addData("Trigger","LEFT");
            frontLeft  =  -fineTurnSpeed;
            frontRight = fineTurnSpeed;
            rearLeft   = -fineTurnSpeed;
            rearRight  =  fineTurnSpeed;
        }
        else if(gamepad1.right_trigger>0.10){
            telemetry.addData("Trigger","RIGHT");
            frontLeft  =  fineTurnSpeed;
            frontRight = -fineTurnSpeed;
            rearLeft   = fineTurnSpeed;
            rearRight  =  -fineTurnSpeed;
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
        if( Math.abs( valueIn) < 0.04 ) {
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

        return valueOut;
    } // multSegLinearRot

    private double multSegLinearXY( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.04 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.33 ) {                       // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.0550;    // 0.01=0.060   0.33=0.1375
            }
            else if( valueIn < 0.90 ) {
                valueOut = (1.00 * valueIn) - 0.1925;   // 0.33=0.1375   0.90=0.7075
            }
            else
                valueOut = (14.0 * valueIn) - 11.8925;  // 0.90=0.7075   1.00=2.1075 (clipped)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.33 ) {
                valueOut = (0.25 * valueIn) - 0.0550;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (1.00 * valueIn) + 0.1925;
            }
            else
                valueOut = (14.0 * valueIn) + 11.8925;
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

} // TeleopRed
