/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toDegrees;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This program implements robot movement based on Gyro heading and encoder counts.
 * It uses the Mecanumbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode and requires:
 * a) Drive motors with encoders
 * b) Encoder cables
 * c) Rev Robotics I2C IMU with name "imu"
 * d) Drive Motors have been configured such that a positive power command moves forward,
 *    and causes the encoders to count UP.
 * e) The robot must be stationary when the INIT button is pressed, to allow gyro calibration.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 */
@Autonomous(name="Autonomous Right-Blue", group="7592", preselectTeleOp = "Teleop-Blue")
//@Disabled
public class AutonomousRightBlue extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    boolean clawOpen = false;
    
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
        sleep( 1000 );

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        redAlliance  = false;
        scorePreloadSpecimen = true;
        spikeSamples = 2;  // herd 2 of 3 (so we have time left to score another specimen
        parkLocation = PARK_OBSERVATION;

        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to preload a specimen?
            if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last) {
                if( clawOpen ) {
                    robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_CLOSED );
                    clawOpen = false;
                } else {
                    robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_OPEN );
                    clawOpen = true;
                }
            } //  gamepad1_r_bumper
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Ensure any movement during robot setup is reset to zero
        resetGlobalCoordinatePosition();

        // Start the autonomous timer so we know how much time is remaining when cycling samples
        autonomousTimer.reset();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
//          pixelNumber = 0;
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
    /*   3 Herd left team sample                                                                  */
    /*   4 Collect specimen from observation zone (2nd preload specimen)                          */
    /*   5 Hang specimen on high rung                                                             */
    /*   6 Herd center team sample                                                                */
    /*   7 Collect specimen from observation zone (from left herd)                                */
    /*   8 Hang specimen on high rung                                                             */
    /*   9 Herd right team sample                                                                 */
    /*   10 Collect specimen form observaiton zone (from center herd)                             */
    /*   11 Hang specimen on high rung                                                            */
    /*   12 Collect specimen from observation zone (from right herd)                              */
    /*   13 Hang specimen on high rung                                                            */
    /*   14 Park in observation zone	                                                          */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        // Score the preloaded SPECIMEN
        if( !onlyPark && scorePreloadSpecimen ) {
            scoreSpecimenPreload();
        }

        if( !onlyPark && (spikeSamples > 0) ) {
            herdSamples(spikeSamples);
        }

        if( !onlyPark && (spikeSamples > 0) ) {
            scoreSpecimenFromWall();
        }

/*
        while (specimensScored < scoreSpecimens) {
            herdSample(specimensScored);
            collectSpecimen();
            scoreSpecimenFromWall(specimensScored);
            specimensScored++;
        } 
*/
        // Park for 3pts (observation zone)
        parkInObservation();
        
        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void scoreSpecimenPreload() {
        // Drive forward to submersible
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to submersible");
            telemetry.update();
            // Move away from field wall (viper slide motor will hit field wall if we tilt up too soon!)
            driveToPosition( 3.00, 0.00, 0.00, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
            // Start tilting and extending the arm, and positioning the specimen
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN1_DEG, 1.0);
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO1);
            // Drive to the scoring position next to the submersible
            driveToPosition( 22.20, -5.90, 0.00, DRIVE_SPEED_60, TURN_SPEED_20, DRIVE_THRU );
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR1);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR1);
            driveToPosition( 28.90, -7.20, 0.00, DRIVE_SPEED_60, TURN_SPEED_20, DRIVE_TO );
            robot.driveTrainMotorsZero();  // make double sure we're stopped
            // If we drive to the submersible faster than the arm moves, wait for the arm
            do {
                if( !opModeIsActive() ) break;
                // only check every 75 msec
                sleep( 75 );
                // update all our status
                performEveryLoop();
            } while( autoTiltMotorMoving() || autoViperMotorMoving());
        } // opModeIsActive

        // Rotate arm, viper slide, and claw down to clip the specimen
        if( opModeIsActive() ) {
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN2_DEG,0.80 );
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO2);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR2);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR2);
            sleep( 1200 ); //while( autoTiltMotorMoving() || autoViperMotorMoving());
            // release the specimen
            robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_OPEN );
        } // opModeIsActive

        // Retract the arm for parking
        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_ZERO);
            driveToPosition( 32.60, 2.70, 52.20, DRIVE_SPEED_80, TURN_SPEED_20, DRIVE_THRU );
        } // opModeIsActivee

        // Now that we're clear from the submersible, rotate arm down and store claw
        if( opModeIsActive() ) {
            robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_CLOSED );
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_INIT);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_INIT);
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL_DEG, 0.80);
            driveToPosition( 38.40, 17.50, 90.00, DRIVE_SPEED_80, TURN_SPEED_20, DRIVE_THRU );
            driveToPosition( 47.40, 10.00, 180.00, DRIVE_SPEED_80, TURN_SPEED_20,
                                               ((spikeSamples > 0)? DRIVE_THRU : DRIVE_TO) );
        } // opModeIsActive

    }  // scoreSpecimenPreload

    private void herdSamples(int samplesToHerd) {
        // Do we herd the first sample on the spike marks?
        if( opModeIsActive() && (samplesToHerd > 0) ) {
            pos_y=61.0; pos_x=10.0; pos_angle=180.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            pos_x+=10.0; // 10" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            pos_y-=38.0; // 38" back toward observation zone
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            if( samplesToHerd == 1 ) { robot.driveTrainMotorsZero(); } // go there fast, but stop
        } // opModeIsActive
        // What about the 2nd sample?
        if( opModeIsActive() && (samplesToHerd > 1) ) {
            pos_y=61.0; pos_x=24.0; pos_angle=180.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            pos_x+=6.0; // 10" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            pos_y-=36.0; // 36" back toward observation zone
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            if( samplesToHerd == 2 ) { robot.driveTrainMotorsZero(); } // go there fast, but stop
        } // opModeIsActive
        // What about the 3rd one against the wall?
        if( opModeIsActive() && (samplesToHerd > 2) ) {
            pos_y=54.0; pos_x=32.0; pos_angle=180.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            timeDriveStrafe(DRIVE_SPEED_20,1250); // ensure we slowly align to the wall
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
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_60, TURN_SPEED_20, DRIVE_THRU );
            // Go fast to the edge of the observation zone
            pos_y =  31.0;
            pos_x -= 4.0;  // end 5" away from the wall
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_80, TURN_SPEED_40, DRIVE_THRU );
            // ease into the observation zone (in case we hit the wall, or another robot)
            timeDriveStraight(DRIVE_SPEED_20,1000);  // this stops all motors
            // NOTE: this ending position also counts as PARKED
        } // opModeIsActive

    } //herdSample

    /*--------------------------------------------------------------------------------------------*/
    private void scoreSpecimenFromWall() {

        // Drive close to the wall-collect position (quickly)
        if( opModeIsActive() ) {
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL0_DEG, 1.0);
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_WALL0);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_WALL0);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_WALL0);
            driveToPosition( 20.0, 9.0, 180, DRIVE_SPEED_50, TURN_SPEED_20, DRIVE_THRU );
            robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_OPEN_WIDE );
        } // opModeIsActive

        // Drive to the final wall-collect position (slowly)
        if( opModeIsActive() ) {
            driveToPosition( 16.2, 7.2, 180, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
            robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_CLOSED );
            sleep(500); // allow claw to close (250msec)
        } // opModeIsActive

        // Lift the specimen off the field wall
        if( opModeIsActive() ) {
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL1_DEG, 1.0);
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_WALL1);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_WALL1);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_WALL1);
            sleep(750); // allow arm to lift above the wall (750 msec)
        } // opModeIsActive

        driveToPosition( 20.0, 7.2, 180, DRIVE_SPEED_50, TURN_SPEED_20, DRIVE_TO );

    } // scoreSpecimenFromWall

    /*--------------------------------------------------------------------------------------------*/
    private void parkInObservation() {
        if( (spikeSamples < 1) && opModeIsActive() ) {
            // Rotate 90deg to face wall (protect collector from alliance partner damage)
            driveToPosition(12.0, 14.0, -91.0, DRIVE_SPEED_50, TURN_SPEED_50, DRIVE_THRU);
            // Park in far corner of observation zone
            driveToPosition(6.0, 32.0, -91.0, DRIVE_SPEED_50, TURN_SPEED_30, DRIVE_TO);
        }
    } // parkInObservation

} /* AutonomousRightBlue */
