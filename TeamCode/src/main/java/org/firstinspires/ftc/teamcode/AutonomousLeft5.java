/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 */
@Autonomous(name="Left3: 5 yellow (80 pts)", group="7592", preselectTeleOp = "Teleop")
//@Disabled
public class AutonomousLeft5 extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    double pos_y=0, pos_x=0, pos_angle=0.0;  // Allows us to specify movement INCREMENTALLY, not ABSOLUTE

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous mode)
        robot.init(hardwareMap,true);

        // Robot starts facing 90deg sideways relative to driver
        robot.rcStartAngleSet( -90.0 );

        // Initialize webcams using OpenCV
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Ensure viper arm is fully retracted before we start
        ensureViperArmFullyRetracted();

        // Ensure snorkels are fully retracted before we start
        ensureSnorkelFullyRetracted(true);
        ensureSnorkelFullyRetracted(false);

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        redAlliance  = true;
        scorePreloadSpecimen = false;
        spikeSamples = 3;
        parkLocation = PARK_SUBMERSIBLE;

        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to preload a specimen?
            if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last) {
                if( clawOpen ) {
                    robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_CLOSED );
                    clawOpen = false;
                } else {
                    robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN );
                    clawOpen = true;
                }
            } //  gamepad1_r_bumper
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu(true);  // yes, use auto5 start position
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Ensure any movement during robot setup is reset to zero
        resetGlobalCoordinatePosition();

        // Start the autonomous timer so we know how much time is remaining when cycling samples
        autonomousTimer.reset();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
//          createAutoStorageFolder(redAlliance, pipelineBack.leftSide);
//          pipelineBack.setStorageFolder(storageDir);
//          pipelineBack.saveSpikeMarkAutoImage();
        }

        //---------------------------------------------------------------------------------
        // UNIT TEST: The following methods verify our basic robot actions.
        // Comment them out when not being tested.
//      testGyroDrive();
//      unitTestOdometryDrive();
//      timeArmMovement();
        //---------------------------------------------------------------------------------

        //---------------------------------------------------------------------------------
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
        // Comment it out if running one of the unit tests above.
        mainAutonomous();
        //---------------------------------------------------------------------------------

        telemetry.addData("Program", "Complete");
        telemetry.update();

    } /* runOpMode() */

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Verify gyro/encoder-based motion functions against a tape measure
    private void testGyroDrive() {
        double startAngle;
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, 12.0, 999.9, DRIVE_THRU ); // Drive FWD 12" along current heading
        gyroDrive(DRIVE_SPEED_20, DRIVE_X, 12.0, 999.9, DRIVE_TO  ); // Strafe RIGHT 12" along current heading
        // What is our starting angle?
        startAngle = getAngle();
        gyroTurn(TURN_SPEED_20, (startAngle + 45) );   // Turn CW 45 degrees
    } // testGyroDrive

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Verify odometry-based motion functions against a tape measure
    private void unitTestOdometryDrive() {
        telemetry.addData("Target", "x=24.0, y=0.0f, 0.00 deg (100%)");
        // reset our timer and drive forward 20"
        autonomousTimer.reset();
        driveToPosition( 24.0, 0.0, 0.0, DRIVE_SPEED_100, TURN_SPEED_80, DRIVE_TO );
        double driveTime = autonomousTimer.milliseconds() / 1000.0;
        performEveryLoop();  // ensure our odometry is updated
        telemetry.addData("Odometry", "x=%.2f, y=%.2f, %.2f deg", robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, Math.toDegrees(robotOrientationRadians) );
        telemetry.addData("Drive Time", "%.3f sec", driveTime );
        telemetry.update();
        sleep(30000);
/*
        telemetry.addData("Target", "x=%.2f, y=%.2f, %.2f deg (100%)", pos_x, pos_y, pos_angle );
*/

/*
        // Drive forward 12"
        driveToPosition( 12.0, 0.0, 0.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
        // Strafe right 12"
        driveToPosition( 12.0, 12.0, 0.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
        // Turn 180 deg
        driveToPosition( 12.0, 12.0, 90.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
        // Report the final odometry position/orientation
        telemetry.addData("Final", "x=%.1f, y=%.1f, %.1f deg",
                robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, toDegrees(robotOrientationRadians) );
        telemetry.update();
        sleep( 7000 );
 */
    } // unitTestOdometryDrive

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Time how long our arm and viper slide take to get to a specified position
    private void timeArmMovement() {
       boolean tiltDone  = false;
       boolean viperDone = false;
       double  tiltTime = 0.0;
       double  viperTime = 0.0;
       // reset our timer
       autonomousTimer.reset();
       // start both movements
       autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN1_DEG, 1.0);
       autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO1);
       // wait for both to finish
       do {
          if( !opModeIsActive() ) break;
          // only check every 100 msec
          sleep( 100 );
          // update all our status
          performEveryLoop();
          if( !tiltDone && !autoTiltMotorMoving() ) {
              tiltTime = autonomousTimer.milliseconds();
              tiltDone = true;
          }
          if( !viperDone && !autoViperMotorMoving() ) {
              viperTime = autonomousTimer.milliseconds();
              viperDone = true;
          }
       } while( !tiltDone || !viperDone );
       // display the results
       telemetry.addData("Tilt",  "%.1f sec", tiltTime/1000.0 );   // 2.2 sec
       telemetry.addData("Viper", "%.1f sec", viperTime/1000,0 );  // 1.2 sec
       telemetry.update();
       sleep( 7000 );
    } // timeArmMovement

    /*--------------------------------------------------------------------------------------------*/
    /* Autonomous Left:                                                                           */
    /*   1 Starting point                                                                         */
    /*   2 Place sample in upper bucket                                                           */
    /*   3 Collect right neutral sample                                                           */
    /*   4 Place sample in upper bucket                                                           */
    /*   5 Collect center neutral sample                                                          */
    /*   6 Place sample in upper bucket                                                           */
    /*   7 Collect left neutral sample                                                            */
    /*   8 Place sample in upper bucket                                                           */
    /*   9 Level one ascent                                                                       */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        // Score the preloaded SAMPLE from a modified starting position
        scoreSamplePreload();
        scoreSampleFromHuman();
        collectSample(1);
        scoreSample(1);
        // Score the 2nd preload SAMPLE from human player
        collectSample(2);
        scoreSample(3); // score(2) is from human player
        collectSample(3);
        scoreSample(3);
        // Park for 3pts (level 1 ascent)
        level1Ascent();

        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();
    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void scoreSamplePreload() {
        // NOTE: This version starts with robot turned sideways to wall, and with lift up higher!
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to submersible");
            telemetry.update();
            // Move away from field wall (viper slide motor will hit field wall if we tilt up too soon!)
            driveToPosition( -5.0, -5.0, 0.0, DRIVE_SPEED_100, TURN_SPEED_70, DRIVE_THRU );
            // Move to basket and score preloaded sample
            scoreSample(0);
        } // opModeIsActive

    } // scoreSamplePreload

    /*--------------------------------------------------------------------------------------------*/
    private void scoreSampleFromHuman() {

        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Drive to human");
            telemetry.update();
            // Drive over to human player to collect yellow sample)
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SUBMERSIBLE_DEG);
            driveToPosition(60.0, -5.0, 0.0, DRIVE_SPEED_100,  TURN_SPEED_70, DRIVE_TO);
            robot.driveTrainMotorsZero();  // make double sure we're stopped
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_COLLECT1_DEG);
            sleep(900);
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_CLOSED );
            sleep(500);
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SUBMERSIBLE_DEG);
            driveToPosition(12, -8.3, 0.0, DRIVE_SPEED_100, TURN_SPEED_70, DRIVE_THRU);
        } // opModeIsActive

        if( opModeIsActive() ) {
            // Move to basket and score sample from human
            scoreSample(2);
        } // opModeIsActive

    } // scoreSampleFromHuman

    //************************************
    // Collect sample
    //************************************
    private void collectSample(int samplesScored) {

        switch(samplesScored) {
            case 1:
                // Drive forward toward the wall
                driveToPosition( -15.9, -20.5, 90.0, DRIVE_SPEED_70, TURN_SPEED_40, DRIVE_TO );
                do {
                    if( !opModeIsActive() ) break;
                    // wait for lift/tilt to finish...
                    sleep( 50 );
                    // update all our status
                    performEveryLoop();
                } while( autoViperMotorMoving() || autoTiltMotorMoving() ); // viper should already be in position
                break;
            case 2:
                driveToPosition( -25.9, -20.8, 90.0, DRIVE_SPEED_70, TURN_SPEED_40, DRIVE_TO );
                do {
                    if( !opModeIsActive() ) break;
                    // wait for lift/tilt to finish...
                    sleep( 50 );
                    // update all our status
                    performEveryLoop();
                } while( autoViperMotorMoving() || autoTiltMotorMoving() );  // wait for viper to fully retract
                break;
            case 3:
                autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_SAMPLE3);
                autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SAMPLE3_DEG, 1.0 );
                robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB3);
                // drive slow because waiting for arm to lower
                driveToPosition( -27.1, -24.7, 90, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_TO );
                do {
                    if( !opModeIsActive() ) break;
                    // wait for lift/tilt to finish...
                    sleep( 50 );
                    // update all our status
                    performEveryLoop();
                } while( autoViperMotorMoving() || autoTiltMotorMoving() );
                driveToPosition( -27.1, -24.7, 120.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_TO );
                break;
            default:
        }

        // Close the claw on this sample
        robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_CLOSED );
        sleep(550); // wait for claw to close on sample
    } // collectSample

    //************************************
    // Score Sample
    //************************************
    private void scoreSample(int samplesScored) {
        int viperError;
        boolean viperMotorStillTooFar;
        autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_BASKET_DEG, 1.0 );
        // drive partway there while we wait for arm to lift (before extending viper)
        if( samplesScored == 0 ){ // Preload
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_BASKET );
            driveToPosition( -23.5, -6.0, 44.8, DRIVE_SPEED_80, TURN_SPEED_50, DRIVE_TO );
            robot.driveTrainMotorsZero();  // make double sure we're stopped
        }  else if( samplesScored == 1 ){ // From spike mark 1
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_BASKET );
            driveToPosition( -23.5, -6.0, 44.8, DRIVE_SPEED_80, TURN_SPEED_50, DRIVE_TO );
        } else if( (samplesScored == 2) ){ // From human
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_BASKET );
            // make sure we swing out away from wall before we turn to 45deg so robot doesn't hit wall
            driveToPosition( -5.0, -11.0, 22.8, DRIVE_SPEED_100, TURN_SPEED_70, DRIVE_THRU );
            driveToPosition( -17.0, -8.3, 44.8, DRIVE_SPEED_80, TURN_SPEED_70, DRIVE_THRU );
            driveToPosition( -23.5, -6.0, 44.8, DRIVE_SPEED_80, TURN_SPEED_50, DRIVE_TO );
        } else if( samplesScored == 3 ){ // From spike marks 2 and 3
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_BASKET );
            driveToPosition( -23.5, -5.9, 44.8, DRIVE_SPEED_80, TURN_SPEED_50, DRIVE_TO );
        }
        robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BASKET);
        robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BASKET1);
        do {
            if( !opModeIsActive() ) break;
            // wait for lift/tilt to finish...
            sleep( 50 );
            // update all our status
            performEveryLoop();
            // We get a big pause because of viper overshoot (from 22 too high to 22 too low)
            viperError = ( robot.viperMotorPos - Hardware2025Bot.VIPER_EXTEND_BASKET );
            viperMotorStillTooFar = Math.abs(viperError) > 27;  // normal targetPositionTolerance is 10 cts
        } while( viperMotorStillTooFar || autoTiltMotorMoving() );
        robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BASKET);
        robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BASKET2);
        sleep(250); // wait for wrist/elbow to move
        robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
        sleep(250); // wait for claw to drop sample
        robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
        robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
        sleep(250); // wait for claw to start moving up/back before lowering arm
        // Prepare arm for the next collect (lower a bit slower so it doesn't bounce down onto samples)
        autoTiltMotorMoveToTarget( (Hardware2025Bot.TILT_ANGLE_SAMPLE3_DEG + 0.1), 0.9);
        autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO_COLLECT);
    } // scoreSample

    private void level1Ascent() {
        if( opModeIsActive() ) {
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SUBMERSIBLE_DEG, 1.0);
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_SAFE);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_INIT);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_INIT);
            // We don't have time to park, but don't let auto just end here or
            // the arm won't have time to retract.  Sleep to prevent the end of auto
            sleep(2000 );
        } // opModeIsActive
    } // level1Ascent

} /* AutonomousLeft5 */
