/* FTC Team 7572 - Version 1.0 (11/11/2023)
*/
package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

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
@Autonomous(name="Autonomous Right-Red", group="7592", preselectTeleOp = "Teleop-Red")
//@Disabled
public class AutonomousRightRed extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    ElapsedTime intakeTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap,true);

        // Initialize webcams using OpenCV
        telemetry.addData("State", "Initializing webcam (please wait)");
        telemetry.update();

        // This is the line that determined what auto is run.
        // This is right side red alliance.
        pipelineBack = new CenterstageSuperPipeline(false, true );
        visionPortalBack = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam Back"))
                .addProcessor(pipelineBack)
                .setCameraResolution(new Size(1280, 800))
                .build();

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        while (!isStarted()) {
            telemetry.addData("ALLIANCE", "%s %c (X=blue O=red)",
                    ((redAlliance)? "RED":"BLUE"), ((forceAlliance)? '*':' '));
            // If vision pipeline disagrees with forced alliance setting, report it
            if( forceAlliance && (redAlliance != pipelineBack.redAlliance) )
               telemetry.addData("WARNING!!", "vision pipeline thinks %s !!!", (pipelineBack.redAlliance)? "RED":"BLUE");
            telemetry.addData("STARTING", "%s", "RIGHT");
            telemetry.addData("TeamProp", " Hue("  + pipelineBack.targetHue +
                                          ") L:"   + pipelineBack.avg1   +
                                          " C:"    + pipelineBack.avg2   + 
                                          " R:"    + pipelineBack.avg3   +
                                          " Zone:" + pipelineBack.spikeMark );
            telemetry.addData("5-stack cycles", "%d", fiveStackCycles );
            telemetry.addLine("   use LEFT/RIGHT bumpers to modify");
            telemetry.update();
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Force RED alliance?
            if( gamepad1_circle_now && !gamepad1_circle_last ) {
                redAlliance = true;  // gamepad circle is colored RED
                forceAlliance = true;
            }
            // Force BLUE alliance?
            else if( gamepad1_cross_now && !gamepad1_cross_last ) {
                redAlliance = false;   // gamepad cross is colored BLUE
                forceAlliance = true;
            }
            // If we've not FORCED a red/blue alliance, report real-time detection
            if( !forceAlliance ) {
                redAlliance = pipelineBack.redAlliance;
            }
            // Change number of 5-stack to attempt?
            if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last ) {
              fiveStackCycles -= 1;
              if( fiveStackCycles < 0 ) fiveStackCycles=0;              
            }
            else if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last ) {
              fiveStackCycles += 1;
              if( fiveStackCycles > 2 ) fiveStackCycles=2;
            }
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Ensure any movement during robot setup is reset to zero
        globalCoordinatePositionReset();
        
        // Start the autonomous timer so we know how much time is remaining for cone cycling
        autonomousTimer.reset();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
            pixelNumber = 0;
            createAutoStorageFolder(redAlliance, pipelineBack.leftSide);
            pipelineBack.setStorageFolder(storageDir);
            spikeMark = pipelineBack.spikeMark;
            pipelineBack.saveSpikeMarkAutoImage();
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
        mainAutonomous( spikeMark );
        //---------------------------------------------------------------------------------

        telemetry.addData("Program", "Complete");
        telemetry.update();

        visionPortalBack.close();
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
        driveToPosition( 12.0, 0.0, 0.0, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_THRU );
        // Strafe right 12"
        driveToPosition( 12.0, 12.0, 0.0, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_THRU );
        // Turn 180 deg
        driveToPosition( 12.0, 12.0, 179.9, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_TO );
    } // unitTestOdometryDrive

    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous( int spikemark ) {
        double pos_y=0, pos_x=0;

        // Drive forward to spike mark
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to Spike Mark");
            telemetry.update();
            // This movement depends on whether it's left/center/right spike (1/2/3)
            switch( spikemark ) {
                case 1 : // LEFT
                    driveToPosition( -11.0, 0.0, 0.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -16.0, -5.0,41.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -30.5, 6.5, 59.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -25.8, 3.7, 79.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -24.6, 3.9, 85.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -24.7, 6.0, 122.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO );
                    break;
                case 2:  // CENTER
                    driveToPosition( -10.0,  0.0,  0.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -20.0, -7.6, 43.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -24.0, -8.2, 86.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -28.0, -3.5, 94.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -37.0, -2.8, 94.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -37.0, -5.0, 94.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO );
                    break;
                case 3:  // RIGHT
                default:
                    driveToPosition(  -7.0,  0.0,   0.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -10.4, -5.5,  21.5, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -14.5, -12.5, 40.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -18.0, -15.3, 51.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -20.7, -14.0, 56.6, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -20.7, -14.0, 56.6, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -25.5, -10.2, 47.3, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -29.4, -6.9, 37.8, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -34.0, -5.0, 11.6, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -40.9, -8.4, 0.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    break;
            } // switch
        }

        // Eject purple pixel
        if( opModeIsActive()) {
            telemetry.addData("Skill", "eject purple pixel");
            telemetry.update();
            robot.collectorMotor.setPower(robot.COLLECTOR_EJECT_POWER);
            sleep(3000 );  // 3 sec
            robot.collectorMotor.setPower(0.0);
        }

        // Drive toward backdrop in preparation to score the yellow pixel
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "move to backdrop");
            telemetry.update();
            switch( spikemark ) {
                case 1 : // LEFT
                    driveToPosition( -24.2,   7.3, 140.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -22.4,   4.9, 136.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -26.7,   7.7, 101.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -32.0, -34.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    pos_y = -32.0;
                    pos_x = -34.0;
                    break;
                case 2:  // CENTER
                    driveToPosition( -35.2, -10.8, 94.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -26.0, -25.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
                    driveToPosition( -26.0, -34.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    pos_y = -26.0;
                    pos_x = -34.0;
                    break;
                case 3:  // RIGHT
                default:
                    driveToPosition( -45.2, -8.4, 0.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -45.2, -12.9, 0.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -39.0, -18.0, 45.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -33.3, -21.8, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
                    driveToPosition( -20.0, -34.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                    pos_y = -20.0;
                    pos_x = -35.0;
                    break;
            } // switch
        }

        // Score the yellow pixel
        if( opModeIsActive() ) {
            double desiredDistanceCM;
            double currentDistanceCM;
            double driveOffsetInches;
            telemetry.addData("Motion", "move to backdrop");
            telemetry.update();
            switch( spikemark ) {
                case 1 : desiredDistanceCM = 13.0; break; // LEFT
                case 2:  desiredDistanceCM = 14.0; break; // CENTER
                case 3:
                default: desiredDistanceCM = 15.0; break; // RIGHT
            } // switch
            currentDistanceCM = robot.getBackdropRange();
            driveOffsetInches = (desiredDistanceCM -currentDistanceCM)/2.54;
            //telemetry.addData("Backdrop Range", "%.1f CM", currentDistanceCM);
            //telemetry.addData("Drive Offset", "%.1f IN", driveOffsetInches);
            //telemetry.update();
            //sleep(3000);
            if( Math.abs(driveOffsetInches) < 7.0 ) {
                pos_x += driveOffsetInches;
                driveToPosition( pos_y, pos_x, 90.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO);
            }
            scoreYellowPixel();
        }

        // Drive to where we can park in the backstage
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "park in back stage");
            telemetry.update();
            //Back away from the backdrop 2 inches.
            pos_x += 2;
            driveToPosition( pos_y, pos_x, 90.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO);
            //Strafe left to park
            driveToPosition( -50.0, pos_x, 90.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_THRU);
            pos_x -= 6;
            driveToPosition( -50.0, pos_x, 90.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO);
        }
    } // mainAutonomous

    /*  HOW TO PARK IN CORNER
    driveToPosition( -15.0, -26.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
    driveToPosition( -16.6, -24.1, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
    driveToPosition(  -5.1, -32.1, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
    driveToPosition(  -4.0, -37.8, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
    */

} /* AutonomousRightRed */
