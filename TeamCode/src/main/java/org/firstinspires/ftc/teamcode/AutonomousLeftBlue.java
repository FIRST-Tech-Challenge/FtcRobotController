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
@Autonomous(name="Autonomous Left-Blue", group="7592", preselectTeleOp = "Teleop-Blue")
//@Disabled
public class AutonomousLeftBlue extends AutonomousBase {

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
        redAlliance  = false;
        spikeSamples = 0;  // until we get that code working...
        parkLocation = PARK_SUBMERSIBLE;

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
//  private void mainAutonomous(int scoreSamples) {
    private void mainAutonomous() {

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        // Score the preloaded specimen
        if( !onlyPark && scorePreloadSpecimen ) {
            scoreSpecimenPreload();
        }

/*
        // Score starting sample
        scoreSample();
        int samplesScored = 1;

        while (samplesScored < scoreSamples) {
            collectSample(samplesScored);
            scoreSample();
            samplesScored++;
        } 
*/
        // Park for 3pts (level 1 ascent)
         level1Ascent();

        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();

    } // mainAutonomous

    private void scoreSpecimenPreload() {
        // Drive forward to submersible
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to submersible");
            telemetry.update();
            // Move away from field wall (viper slide motor will hit field wall if we tilt up too soon!)
            pos_y=3.0; pos_x=0.0; pos_angle=0.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_50, TURN_SPEED_20, DRIVE_THRU );
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_AUTO1_DEG);
            pos_y += 3.0;
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_70, TURN_SPEED_20, DRIVE_THRU );
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_BAR1);
            robot.wristServo.setPosition(robot.WRIST_SERVO_BAR1);
            pos_y += 3.0;
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_70, TURN_SPEED_20, DRIVE_THRU );
            robot.elbowServo.setPosition(robot.ELBOW_SERVO_BAR2);
            robot.wristServo.setPosition(robot.WRIST_SERVO_BAR2);
            // small shift right in preparation for 45deg rotation (minimize shift toward alliance partner)
            pos_x += 3.0;
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_70, TURN_SPEED_20, DRIVE_TO );
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
            pos_y += 23.0;  pos_x += 3.5; pos_angle = 45.0;
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_70, TURN_SPEED_50, DRIVE_TO );
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
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_AUTO2_DEG);
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
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_ZERO_DEG);
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 150 );
                // update all our status
                performEveryLoop();
            } while( autoTiltMotorMoving() );
        } // opModeIsActive

    } // scoreSpecimenPreload

/*
    private void scoreSample() {
        // TODO: FILL IN IMPLEMENTATION
    } // scoreSample

    private void collectSample(int samplesScored) {
        // TODO: FILL IN IMPLEMENTATION
    } // collectSample
*/

    private void level1Ascent() {
        if( opModeIsActive() ) {
            // Back up from submersible
            driveToPosition( 32.0, 6.0, 90.0, DRIVE_SPEED_50, TURN_SPEED_50, DRIVE_TO );
            // Drive forward toward the wall
            driveToPosition( 38.0, -27.0, 90.0, DRIVE_SPEED_50, TURN_SPEED_30, DRIVE_TO );
        } // opModeIsActive

        if( opModeIsActive() ) {
            // Strafe towards submersible
            driveToPosition( 64.0, -27.0, 90.0, DRIVE_SPEED_70, TURN_SPEED_50, DRIVE_TO );
            // Drive backward
            driveToPosition( 64.0, -15.0, 90.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
        } // opModeIsActive

        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget( robot.VIPER_EXTEND_GRAB);
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_BASKET_DEG);
            timeDriveStraight(-DRIVE_SPEED_20,3000);
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 150 );
                // update all our status
                performEveryLoop();
            } while( autoTiltMotorMoving() || autoViperMotorMoving() );
        } // opModeIsActive

    } // level1Ascent

} /* AutonomousLeftBlue */
