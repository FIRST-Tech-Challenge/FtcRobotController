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

    boolean geckoServoCollecting = false;
    
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
        parkLocation = PARK_NONE;  // no parking, observation zone CORNER, observation zone TRIANGLE or Submersible
        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to preload a specimen?
            if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last) {
                if (geckoServoCollecting) {
                    robot.geckoServo.setPower(0.00);  // toggle collect OFF
                    geckoServoCollecting = false;
                } else {
                    robot.geckoServo.setPower(-0.50); // toggle collect ON
                    geckoServoCollecting = true;
                }
            }
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
//  private void mainAutonomous(int scoreSpecimens) {
    private void mainAutonomous() {

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        // Score the preloaded specimen
        if( !onlyPark && scorePreloadSpecimen ) {
            scoreSpecimenPreload();
        }


        if( !onlyPark && (spikeSamples > 0) ) {
            herdSample(spikeSamples);
        }
/*
        // Score starting specimen
        scoreSpecimen(specimensScored);
        int specimensScored = 1;

        while (specimensScored < scoreSpecimens) {
            herdSample(specimensScored);
            collectSpecimen();
            scoreSpecimen(specimensScored);
            specimensScored++;
        } 
*/
        // Park for 3pts (observation zone)
        parkInObservation();
        
        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();

    } // mainAutonomous

    private void scoreSpecimenPreload() {
        // Drive forward to submersible
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to submersible");
            telemetry.update();
            // Move away from field wall (viper slide motor will hit field wall if we tilt up too soon!)
            driveToPosition( 3.0, 0.0, 0.0, DRIVE_SPEED_50, TURN_SPEED_20, DRIVE_THRU );
            autoTiltMotorMoveToTarget( robot.TILT_ANGLE_AUTO1);
            driveToPosition( 6.0, 0.0, 0.0, DRIVE_SPEED_70, TURN_SPEED_20, DRIVE_THRU );
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_BAR1);
            robot.wristServo.setPosition(robot.WRIST_SERVO_BAR1);
            driveToPosition( 9.0, 0.0, 0.0, DRIVE_SPEED_70, TURN_SPEED_20, DRIVE_THRU );
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_BAR2);
            robot.wristServo.setPosition(robot.WRIST_SERVO_BAR2);
            // approach submersible away from alliance partner
            driveToPosition( 20.0, -2.2, 0.0, DRIVE_SPEED_70, TURN_SPEED_20, DRIVE_TO );
            robot.driveTrainMotorsZero();
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 150 );
                // update all our status
                performEveryLoop();
            } while( autoTiltMotorMoving() );
        } // opModeIsActive

        if( opModeIsActive() ) {
            driveToPosition( 32.0, -2.2, 45.0, DRIVE_SPEED_70, TURN_SPEED_50, DRIVE_TO );
            robot.driveTrainMotorsZero();
            autoViperMotorMoveToTarget( robot.VIPER_EXTEND_AUTO1);
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 200 );
                // update all our status
                performEveryLoop();
            } while( autoViperMotorMoving() );
        } // opModeIsActive

        // Rotate lift down to get specimen close to bar
        if( opModeIsActive() ) {
            robot.geckoServo.setPower(-0.50); // hold it while we clip
            autoTiltMotorMoveToTarget( robot.TILT_ANGLE_AUTO2);
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 150 );
                // update all our status
                performEveryLoop();
            } while( autoTiltMotorMoving() );
        } // opModeIsActive

        // Retract lift to clip the specimen on the bar
        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget( robot.VIPER_EXTEND_AUTO2);
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 200 );
                // update all our status
                performEveryLoop();
            } while( autoViperMotorMoving() );
        } // opModeIsActive

        // Release the specimen once its clipped
        if( opModeIsActive() ) {
            robot.geckoServo.setPower(0.25); // release
            sleep( 750 );
            robot.geckoServo.setPower(0.0); // stop
        } // opModeIsActive

        // Retract the arm for parking
        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget( robot.VIPER_EXTEND_ZERO);
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 200 );
                // update all our status
                performEveryLoop();
            } while( autoViperMotorMoving() );
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_INIT);
            robot.wristServo.setPosition(robot.WRIST_SERVO_INIT);
        } // opModeIsActive

        // Lower the arm for parking
        if( opModeIsActive() ) {
            autoTiltMotorMoveToTarget( robot.TILT_ANGLE_ZERO);
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 150 );
                // update all our status
                performEveryLoop();
            } while( autoTiltMotorMoving() );
            // Back up from submersible
            driveToPosition( 30.0, 14.0, 45.0, DRIVE_SPEED_70, TURN_SPEED_50, DRIVE_TO );
        } // opModeIsActive

    } // scoreSpecimenPreload

    private void herdSample(int samplesToHerd) {
        // Do we herd the first specimen?
        if( opModeIsActive() && (samplesToHerd > 0) ) {
            pos_y=30.0; pos_x=20.0; pos_angle=0.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            pos_y+=18.0; // 18" away from observation zone
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            pos_x+=10.0; // 10" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            pos_y-=38.0; // 38" back toward opbservation zone
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
        } // opModeIsActive
        // What about the 2nd?
        if( opModeIsActive() && (samplesToHerd > 1) ) {
            pos_y=48.0; pos_x=34.0; pos_angle=0.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            pos_x+=6.0; // 10" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            pos_y-=38.0; // 38" back toward opbservation zone
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
        } // opModeIsActive
        // What about the 3rd one against the wall?
        if( opModeIsActive() && (samplesToHerd > 2) ) {
            pos_y=45.0; pos_x=44.0; pos_angle=0.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU );
            timeDriveStrafe(-DRIVE_SPEED_20,1500); // ensure we slowly align to the wall
            // What does odometry report as our new X,Y location? (we're aligned to wall, so by
            // definition we're at 0deg, even if the initial alignment was off a degree or two
            pos_y=robotGlobalYCoordinatePosition;
            pos_x=robotGlobalXCoordinatePosition;
            robotOrientationRadians = 0.0;
            // Drive away from the wall in a DIAGONAL FORWARD movement (driving  sideways away
            // from the wall might leave our magnetic sign attached to the field wall!
            pos_y -= 2.0;
            pos_x -= 2.0;
            pos_angle=-5.0;  // angle the bot away from the wall as we herd the final sample
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_60, TURN_SPEED_40, DRIVE_THRU );
            // Go fast to the edge of the observation zone
            pos_y =  14.0;
            pos_x -= 3.0;  // end 5" away from the wall
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_80, TURN_SPEED_40, DRIVE_THRU );
            // ease into the observation zone (in case we hit the wall, or another robot)
            timeDriveStraight(-DRIVE_SPEED_20,1500);
        } // opModeIsActive
        // If we did any herding, turn off the motors
        if (samplesToHerd > 0) {
            robot.driveTrainMotorsZero();
        }

    } //herdSample

/*
    private void collectSpecimen() {
        // TODO: FILL IN IMPLEMENTATION
    } // collectSpecimen
*/

    private void parkInObservation() {
        if (spikeSamples < 1) {
            // Rotate 90deg to face wall (protect collector from alliance partner damage)
            driveToPosition(12.0, 14.0, -91.0, DRIVE_SPEED_50, TURN_SPEED_50, DRIVE_TO);
            // Park in far corner of observation zone
            driveToPosition(6.0, 32.0, -91.0, DRIVE_SPEED_50, TURN_SPEED_30, DRIVE_TO);
        }
    } // parkInObservation

} /* AutonomousRightBlue */
