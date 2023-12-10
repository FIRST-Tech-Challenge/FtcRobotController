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
@Autonomous(name="Autonomous Left-Red", group="7592", preselectTeleOp = "Teleop-Left")
@Disabled
public class AutonomousLeftRed extends AutonomousBase {

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
        // This is left side red alliance.
        pipelineBack = new CenterstageSuperPipeline(true, true );
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
            telemetry.addData("STARTING", "%s", "LEFT");
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
            createAutoStorageFolder(redAlliance, true);
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

     // Drive forward to spike mark
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to Spike Mark");
            telemetry.update();
            // THe final motion depends on whether it's left/center/right spike (1/2/3)
            switch( spikemark ) {

                // NOTE: we need to go NEGATIVE Y compared to what we did on the blue side (since -90 instead of +90)

                case 1 : // LEFT
                    driveToPosition( -9.0, 0.0, 0.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU );
                    driveToPosition( -16.0, 0.0, 90.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU );
                    driveToPosition( -30.0, 0.0, 90.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU);
                    driveToPosition( -21.0, -4.0, 160.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_TO);
                    break;
                case 2:  // CENTER
                    driveToPosition( -10.0, -3.0, 0.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU );
                    driveToPosition( -37.0, -6.0, 90.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_TO);
                    break;
                case 3:  // RIGHT
                default:
                    driveToPosition( -10.0, 0.0, 0.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU );
                    driveToPosition( -15.0, 1.0, -135.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU );
                    driveToPosition( -25.4, 11.0, -135.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU);
                    driveToPosition( -23.0, 8.0, -143.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_TO);
                    break;
            } // switch
        }

        // Eject purple pixel
        if( opModeIsActive()) {
            telemetry.addData("Skill", "eject purple pixel");
            telemetry.update();
            robot.collectorMotor.setPower(robot.COLLECTOR_EJECT_POWER);
            sleep(2500 );  // 2.5 sec
            robot.collectorMotor.setPower(0.0);
        }

        // Navigate back to channel 1 (avoid alliance partner's pixel in channel 2)
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "navigate to channel 1");
            telemetry.update();
            switch( spikemark ) {
                case 1 : // LEFT
                    driveToPosition( -8.0, -1.0, 135.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU);
                    driveToPosition( -1.0, 20.0, 90.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_TO);
                    break;
                case 2:  // CENTER
                    driveToPosition( -20.0, -8.0, -90.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU);
                    driveToPosition( -8.0, -1.0, 45.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU);
                    driveToPosition( -1.0, 20.0, 90.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_TO);
                    break;
                case 3:  // RIGHT
                default:
                    driveToPosition( -20.0, 6.0, -150.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU );
                    driveToPosition( -18.0, 3.0, -178.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU );
                    driveToPosition( -9.0,  10.0, 130.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_THRU);
                    driveToPosition( -4.0, 20.0, 90.0, DRIVE_SPEED_40, TURN_SPEED_30, DRIVE_TO);
                    break;
            } // switch
        }

        // Park in back stage
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "park in back stage");
            telemetry.update();
            // FOR NOW, CAN'T PARK FROM RIGHT SIDE
        }

    } // mainAutonomous

} /* AutonomousRightBlue */
