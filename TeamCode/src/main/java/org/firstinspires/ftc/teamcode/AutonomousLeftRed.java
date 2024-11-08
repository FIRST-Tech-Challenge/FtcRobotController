/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toDegrees;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
@Autonomous(name="Autonomous Left-Red", group="7592", preselectTeleOp = "Teleop-Red")
//@Disabled
public class AutonomousLeftRed extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap,true);

        // Initialize webcams using OpenCV
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        parkLocation = PARK_NONE;  // no parking, observation zone CORNER, observation zone TRIANGLE or Submersible
        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Ensure any movement during robot setup is reset to zero
//      setGlobalCoordinatePosition(0.0, 0.0, 0.0);
//      setCorrectedGlobalCoordinatePosition(0.0, 0.0, 0.0);

        // Start the autonomous timer so we know how much time is remaining for cone cycling
        autonomousTimer.reset();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
//          pixelNumber = 0;
//          createAutoStorageFolder(redAlliance, pipelineBack.leftSide);
//          pipelineBack.setStorageFolder(storageDir);
//          spikeMark = pipelineBack.spikeMark;
//          pipelineBack.saveSpikeMarkAutoImage();
        }

        //---------------------------------------------------------------------------------
        // UNIT TEST: The following methods verify our basic robot actions.
        // Comment them out when not being tested.
//      testGyroDrive();
        unitTestOdometryDrive();
        //---------------------------------------------------------------------------------

        //---------------------------------------------------------------------------------
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
        // Comment it out if running one of the unit tests above.
//      mainAutonomous();
        //---------------------------------------------------------------------------------

        telemetry.addData("Program", "Complete");
        telemetry.update();

    } /* runOpMode() */

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Verify gyro/encoder-based motion functions against a tape measure
    private void testGyroDrive() {
        double startAngle;
        gyroDrive(DRIVE_SPEED_50, DRIVE_Y, 24.0, 999.9, DRIVE_THRU ); // Drive FWD 24" along current heading
        gyroDrive(DRIVE_SPEED_50, DRIVE_X, 24.0, 999.9, DRIVE_THRU ); // Strafe RIGHT 24" along current heading
        gyroDrive(DRIVE_SPEED_50, DRIVE_Y, -24.0, 999.9, DRIVE_THRU);
        gyroDrive(DRIVE_SPEED_50, DRIVE_X, -24.0, 999.9, DRIVE_THRU);
        // What is our starting angle?
        startAngle = getAngle();
        gyroTurn(TURN_SPEED_80, (startAngle + 120.0) );   // Turn CW 120 degrees
        gyroTurn(TURN_SPEED_80, (startAngle + 240.0) );   // Turn another 120 degrees (240 total)
        gyroTurn(TURN_SPEED_80, startAngle );             // Turn back to starting angle (360 total)
    } // testGyroDrive

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Verify odometry-based motion functions against a tape measure
    private void unitTestOdometryDrive() {
        // Drive forward 12"
        driveToPosition( 12.0, 0.0, 0.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_THRU );
        // Strafe right 12"
        driveToPosition( 12.0, 12.0, 0.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_THRU );
        // Turn 180 deg
        driveToPosition( 12.0, 12.0, 90.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
        // Report the final odometry position/orientation
        telemetry.addData("Final", "x=%.1f, y=%.1f, %.1f deg",
                robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, toDegrees(robotOrientationRadians) );
        telemetry.update();
        sleep( 7000 );
    } // unitTestOdometryDrive

    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {
/*
        double pos_y=0, pos_x=0, pos_angle=-90.0;
        int backdropAprilTagID = 2; // default to BLUE CENTER

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

     // Drive forward to spike mark
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to Spike Mark");
            telemetry.update();
            // This movement depends on whether it's left/center/right spike (1/2/3)
            switch( spikemark ) {
                case 3 : // RIGHT
                    driveToPosition( -10.0, 0.0, 0.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -12.5, -0.5, -67.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -15.0, -1.0, -135.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -25.4, -11.0, -135.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -23.0, -9.5, -143.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    break;
                case 2:  // CENTER
                    driveToPosition( -10.0, 3.0, 0.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -37.0, 6.0, -90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    break;
                case 1:  // LEFT
                default:
                    driveToPosition( -9.0, 0.0, 0.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -16.0, 0.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -30.0, 0.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -25.0, 4.0, 160.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    break;
            } // switch
        }

        // Eject purple pixel
        if( opModeIsActive()) {
            telemetry.addData("Skill", "eject purple pixel");
            telemetry.update();
            // Back straight up for 0.85 sec to drop purple pixel on the spike mark line
            timeDriveStraight( -0.20, 850 );
        }

        // Navigate back to channel 1 (avoid alliance partner's pixel in channel 2)
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "navigate to channel 1");
            telemetry.update();
            switch( spikemark ) {
                case 3 : // RIGHT
                    driveToPosition( -20.0, -4.0, -150.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -19.0, -1.0, -150.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -15.0, -1.0, -178.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -13.0, -1.0, 175.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -8.0, -3.0, 160.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -6.0,  -6.0, 140.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -4.0, -20.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    pos_y = -4.0;
                    break;
                case 2:  // CENTER
                    driveToPosition( -30.0, 12.0, -90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -20.0, 8.0, -90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -8.0, 1.0, 45.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -1.0, -20.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    pos_y = -1.0;
                    break;
                case 1:  // LEFT
                default:
                    driveToPosition( -12.0, 4.0, 150.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -4.0, 2.0, 135.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -1.0, -20.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    pos_y = -1.0;
                    break;
            } // switch
            // Do we pause here under the truss?
            if( trussDelaySec > 0 ) {
                sleep( trussDelaySec * 1000 );
            }
        } // opModeIsActive

        // Drive into back stage area
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Drive into back stage");
            telemetry.update();
            if( audienceYellow || (parkLocation != PARK_NONE) ) {
                driveToPosition( pos_y-1, -60.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                driveToPosition( pos_y-2, -70.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
            } else {
                // no yellow pixel and PARK_NONE means do nothing
            }
        } // opModeIsActive

        // Align to backdrop to score yellow pixel?
        if( opModeIsActive() && audienceYellow ) {
            telemetry.addData("Motion", "Align to backdrop");
            telemetry.update();
            switch( spikemark ) {
                // TODO: Audience side not repeatable enough to for 3" accuracy (odometry depends on starting alignment!)
                // Once AprilTag navigation correction in place, then refine these numbers
                case 1 : pos_y -= ((yellowOnLeft)? 27.0:27.0); pos_x = -84.0; break; // LEFT
                case 2:  pos_y -= ((yellowOnLeft)? 23.0:23.0); pos_x = -84.0; break; // CENTER
                case 3:  pos_y -= ((yellowOnLeft)? 22.0:22.0); pos_x = -84.0; break; // RIGHT
                default: pos_y -= ((yellowOnLeft)? 23.0:23.0); pos_x = -84.0; break; // (CENTER)
            } // switch
            pos_angle = 90.0; // same for all 3 positions
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
            pos_x -= 6.0;  // we're roughly aligned; drive closer;
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO);
        } // opModeIsActive

        // Score yellow pixel
        if( opModeIsActive() && audienceYellow ) {
            double desiredDistanceCM;
            double currentDistanceCM;
            double driveOffsetInches;
            telemetry.addData("Motion", "Score yellow pixel");
            telemetry.update();
            switch( spikemark ) {
                case 3 : desiredDistanceCM = 13.0; break; // RIGHT
                case 2:  desiredDistanceCM = 12.0; break; // CENTER
                case 1:
                default: desiredDistanceCM = 12.0; break; // LEFT
            } // switch
            currentDistanceCM = 6; // robot.getBackdropRange();
            driveOffsetInches = (desiredDistanceCM-currentDistanceCM)/2.54;
//          telemetry.addData("Backdrop Range", "%.1f CM", currentDistanceCM);
//          telemetry.addData("Drive Offset", "%.1f IN", driveOffsetInches);
//          telemetry.update();
//          sleep(3000);
            if( Math.abs(driveOffsetInches) < 7.0 ) {
                pos_x += driveOffsetInches;
                driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO);
            }
            scoreYellowPixel();
        } // opModeIsActive

        // Park in back stage
        if( opModeIsActive() && (parkLocation != PARK_NONE) ) { // Either PARK_LEFT or PARK_RIGHT does the same thing
            telemetry.addData("Motion", "park in back stage");
            telemetry.update();
            // Are we parking from the backdrop or not?
            if( audienceYellow) {
               // Just back away from backdrop a bit
                driveToPosition( pos_y, pos_x+2, pos_angle, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO);
            }
            else { // just finish the drive from the truss
                driveToPosition( pos_y-1, -85.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
            }
        } // opModeIsActive
*/
    } // mainAutonomous

} /* AutonomousLeftRed */
