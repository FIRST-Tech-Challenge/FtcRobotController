/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toDegrees;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Right: clip 4 (73 pts)", group="7592", preselectTeleOp = "Teleop")
//@Disabled
public class AutonomousRight4 extends AutonomousBase {

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
        scorePreloadSpecimen = true;
        spikeSamples = 2;  // herd 2 of 3 (so we have time left to score another specimen
        parkLocation = PARK_OBSERVATION;

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
            processAutonomousInitMenu(false);
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
    /* Autonomous Right:                                                                          */
    /*   1 Starting Point                                                                         */
    /*   2 Hang pre-load specimen at submersible                                                  */
    /*   3 Herd samples from spike marks (left/center/wall)                                       */
    /*   4 Grab clipped specimen from observation zone wall                                       */
    /*   5 Hang specimen on high rung (repeat steps 4 & 5)                                        */
    /*   6 Park in observation zone                                                              */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {
        int specimensHooked  = 0;
        int specimensGrabbed = 0;

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        // Score the preloaded SPECIMEN
        if( !onlyPark && scorePreloadSpecimen ) {
            hookSpecimenOnBarFWD( specimensHooked++ );
        }

        if( !onlyPark && (spikeSamples > 0) ) {
            herdSamples(spikeSamples);
        }

        if( !onlyPark && (spikeSamples > 0) ) {
            grabSpecimenFromWall( specimensGrabbed++ );
            hookSpecimenOnBarBWD( specimensHooked++ );
        }

        if( !onlyPark && (spikeSamples > 1) ) {
            grabSpecimenFromWall( specimensGrabbed++ );
            hookSpecimenOnBarBWD( specimensHooked++ );
        }

        // Retrieve and score the 2nd preload
        grabSpecimenFromWall( specimensGrabbed++ );
        //hookSpecimenOnBarFWD( specimensHooked++ );
        hookSpecimenOnBarBWD( specimensHooked++ );

        // Park for 3pts (observation zone)
        parkInObservation();
        
        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void hookSpecimenOnBarFWD(int specimenNumber ) {

        telemetry.addData("Motion", "Move to submersible");
        telemetry.update();

        // If this is the initial preloaded specimen, inch forward away from the wall
        // (raising the lift while against the wall will cause lift motor to hit rear wall)
        if( opModeIsActive() ) {
            if( specimenNumber == 0 ) {
              // Move away from field wall (viper slide motor will hit field wall if we tilt up too soon!)
              driveToPosition( 3.00, 0.00, 0.00, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_THRU );
              pos_x = -6.25; // hang initial specimen 6.8 inches to the left of starting position
            }
            else { // NEVER USED SINCE WE REVERSE-SCORE ALL OTHER SPECIMENS
              pos_x = -6.25 - (3.0 * specimenNumber); // shift left 3" each time
            }
        } // opModeIsActive

        // Drive toward submersible
        if( opModeIsActive() ) {
            // Start tilting and extending the arm, and positioning the specimen
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN1_DEG, 1.0);
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO1);
            // Drive to the scoring position next to the submersible
            driveToPosition( 18.2, (pos_x+2.2), 0.00, DRIVE_SPEED_55, TURN_SPEED_40, DRIVE_THRU );
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR1);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR1);
            pos_y = 27.80 + (specimenNumber * 0.25);  // specimenNumber doesn't matter, as only use once
            driveToPosition( pos_y, pos_x, 0.00, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_TO );
            robot.driveTrainMotorsZero();  // make double sure we're stopped
            // If we drive to the submersible faster than the arm moves, wait for the arm
            //sleep(100);
        } // opModeIsActive

        // Rotate arm, viper slide, and claw down to clip the specimen
        if( opModeIsActive() ) {
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN2_DEG,0.80 );
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO2);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR2);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR2);
            sleep( 1000 ); //while( autoTiltMotorMoving() || autoViperMotorMoving());
            // release the specimen
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
        } // opModeIsActive

        // Retract the arm for driving as we pivot away from the submersible
        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_ZERO);
            if( specimenNumber == 0 ) { // Back away in preparation of herding
                driveToPosition(26.5, 0.0, 90.0, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU);
            }
            else if (specimenNumber < spikeSamples) { // Back away in preparation to grab more off wall
                driveToPosition(16.0, 3.0, 90.0, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU);
            } else { // Back away in preparation to go park
                driveToPosition(21.0, 3.0, 45.0, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU);
            }
        } // opModeIsActivee

        // Now that we're clear from the submersible, rotate arm down and store claw
        if( opModeIsActive() ) {
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_INIT);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_INIT);
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL_DEG, 0.80);
        } // opModeIsActive

    }  // hookSpecimenOnBar

    private void hookSpecimenOnBarBWD(int specimenNumber ) {

        telemetry.addData("Motion", "Move to submersible");
        telemetry.update();

        // If this is the initial preloaded specimen, inch forward away from the wall
        // (raising the lift while against the wall will cause lift motor to hit rear wall)
        if( opModeIsActive() ) {
            if( specimenNumber == 0 ) { // NEVER USED; FIRST SPECIMEN IS FWD-SCORED
                // Move away from field wall (viper slide motor will hit field wall if we tilt up too soon!)
                driveToPosition( 3.00, 0.00, 0.00, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_THRU );
                pos_x = -6.80; // hang initial specimen 6.8 inches to the left of starting position
            }
            else {
                // Make initial movement toward submersible from wall
                pos_x = -6.80 - (3.0 * specimenNumber); // shift left 3" each time
            }
        } // opModeIsActive

        // Drive toward submersible
        if( opModeIsActive() ) {
            // Start tilting and extending the arm, and positioning the specimen
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN3_DEG, 1.0);
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO3);
            // Drive partway in Y and X toward the scoring position next to the submersible
            driveToPosition( 18.2, (pos_x+10.0), 180.0, DRIVE_SPEED_70, TURN_SPEED_50, DRIVE_THRU );
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR3);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR3);
            // Finish the drive to the submersible bar
            pos_y = 28.40 + (specimenNumber * 0.15);
            driveToPosition( pos_y, pos_x, 180.0, DRIVE_SPEED_50, TURN_SPEED_50, DRIVE_TO );
            robot.driveTrainMotorsZero();  // make double sure we're stopped
        } // opModeIsActive

        // Rotate arm, viper slide, and claw down to clip the specimen
        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO4);
            sleep( 800 );
            // release the specimen
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
        } // opModeIsActive

        // Retract the arm for driving as we pivot away from the submersible
        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_ZERO);
            if( specimenNumber == 0 ) { // Back away in preparation of herding
                driveToPosition(26.5, 0.0, 90.0, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU);
            }
            /*
            else if (specimenNumber < spikeSamples) { // Back away in preparation to grab more off wall
                driveToPosition(16.0, 3.0, 90.0, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU);
            } else { // Back away in preparation to go park
                driveToPosition(21.0, 3.0, 45.0, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU);
            }
             */
        } // opModeIsActivee

        // Now that we're clear from the submersible, rotate arm down and store claw
        if( opModeIsActive() ) {
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_INIT);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_INIT);
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL_DEG, 0.80);
        } // opModeIsActive

    }  // hookSpecimenOnBar

    private void herdSamples(int samplesToHerd) {
        // Do we herd the first sample on the spike marks?
        if( opModeIsActive() && (samplesToHerd > 0) ) {
            // Navigate around the corner of the submersible
            driveToPosition( 26.7, 7.5, 112.0, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU );
            driveToPosition( 30.3, 15.4, 137.0, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU );
            driveToPosition( 34.0, 20.0, 180.00, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU );
            // Align for the first sample
            pos_y=46.5; pos_x=23.7; pos_angle=180.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_70, DRIVE_THRU );
            pos_x+=7.0; // 7" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_70, DRIVE_THRU );
            pos_y-=27.5; // 27.5" back toward observation zone
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_70, DRIVE_THRU );
//          herdForwardQuickly( pos_y, pos_x, pos_angle, DRIVE_SPEED_100 );
            if( samplesToHerd == 1 ) { robot.driveTrainMotorsZero(); } // go there fast, but stop
        } // opModeIsActive
        // What about the 2nd sample?
        if( opModeIsActive() && (samplesToHerd > 1) ) {
            pos_y=45.0; pos_x=32.5; pos_angle=180.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_THRU );
            pos_x+=7.0; // 7" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_THRU );
            pos_y-=27.5; // 27.5" back toward observation zone
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_THRU );
//          herdForwardQuickly( pos_y, pos_x, pos_angle, DRIVE_SPEED_100 );
            if( samplesToHerd == 2 ) { robot.driveTrainMotorsZero(); } // go there fast, but stop
        } // opModeIsActive
        // What about the 3rd one against the wall?
        if( opModeIsActive() && (samplesToHerd > 2) ) {
            pos_y=46.0; pos_x=43.0; pos_angle=180.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_THRU );
            pos_x+=1.5; // 1.5" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_THRU );
            timeDriveStrafe(DRIVE_SPEED_20,1000); // ensure we slowly align to the wall
            // What does odometry report as our new X,Y location? (we're aligned to wall, so by
            // definition we're at 180deg, even if the initial alignment was off a degree or two
            pos_y=robotGlobalYCoordinatePosition;
            pos_x=robotGlobalXCoordinatePosition;
            robotOrientationRadians = Math.toRadians(180.0);
            // Drive away from the wall in a DIAGONAL FORWARD movement (driving  sideways away
            // from the wall might leave our magnetic sign attached to the field wall!
            pos_y -= 5.0;
            pos_x -= 1.0;
            pos_angle=175.0;  // angle the bot away from the wall as we herd the final sample
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_20, DRIVE_THRU );
            // Go fast to the edge of the observation zone
            pos_y =  17.0;
            pos_x -= 2.0;  // end 2" away from the wall
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_40, DRIVE_THRU );
            // ease into the observation zone (in case we hit the wall, or another robot)
            timeDriveStraight(DRIVE_SPEED_20,1000);  // this stops all motors
            // NOTE: this ending position also counts as PARKED
        } // opModeIsActive

    } //herdSample

    /*--------------------------------------------------------------------------------------------*/
    private void grabSpecimenFromWall( int specimenNumber ) {

        // Prepare arm for grabbing specimens from wall, and move to initial wall-collect position (quickly)
        if( opModeIsActive() ) {
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL0_DEG, 1.0);
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_WALL0);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_WALL0);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_WALL0);
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
            if( specimenNumber == 0 ) {
               // Approach along x-axis from herding spike marks...
               driveToPosition( 8.0, 28.0, 180, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU );
            } else {
               // Approach along y-axis from hooking at submersible...
               driveToPosition( 9.5, 13.2, 180, DRIVE_SPEED_100, TURN_SPEED_60, DRIVE_THRU );
            }
        } // opModeIsActive

        // Drive to the final wall-collect position (slowly)
        if( opModeIsActive() ) {
            driveToPosition( 4.6, 18.6, 180, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_TO );
            robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_CLOSED );
            sleep(350); // allow claw to close (350msec)
        } // opModeIsActive

        // Lift the specimen off the field wall
        if( opModeIsActive() ) {
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL1_DEG, 1.0);
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_WALL1);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_WALL1);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_WALL1);
            sleep(600); // allow arm to lift above the wall (600 msec)
        } // opModeIsActive

    } // grabSpecimenFromWall

    /*--------------------------------------------------------------------------------------------*/
    private void parkInObservation() {
        if( opModeIsActive() ) {
            // Park in far corner of observation zone
            driveToPosition(8.0, 30.0,  90.0, DRIVE_SPEED_100, TURN_SPEED_80, DRIVE_TO);
        }
    } // parkInObservation

} /* AutonomousRight4 */
