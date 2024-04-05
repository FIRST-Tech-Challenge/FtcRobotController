/* FTC Team 7572 - Version 1.0 (11/11/2023)
*/
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.THREADS_DEFAULT;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
@Autonomous(name="Autonomous Rescore", group="7592", preselectTeleOp = "Teleop-Red")
//@Disabled
public class AutonomousRescore extends AutonomousBase {

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
//        aprilTag = new AprilTagProcessor.Builder()
//                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                .setLensIntrinsics(904.214,904.214,696.3,362.796)
//                .build();
        aprilTag = new AprilTagProcessorImplCallback(904.214, 904.214, 696.3, 362.796,
                DistanceUnit.INCH, AngleUnit.DEGREES, AprilTagGameDatabase.getCenterStageTagLibrary(),
                false, false, true, true,
                AprilTagProcessor.TagFamily.TAG_36h11, THREADS_DEFAULT, false,
                robotGlobalCoordinateCorrectedPosition);
        visionPortalBack = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(pipelineBack, aprilTag)
                .setCameraResolution(new Size(1280, 800))
                .build();
        //Ensure the camera is in automatic exposure control
        setWebcamAutoExposure();

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        parkLocation = PARK_LEFT;  // red-right normally parks on the left
        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Ensure any movement during robot setup is reset to zero
        setGlobalCoordinatePosition(0.0, 0.0, 0.0);
        setCorrectedGlobalCoordinatePosition(0.0, 0.0, 0.0);

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
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
        // Comment it out if running one of the unit tests above.
        mainAutonomous( spikeMark );
        //---------------------------------------------------------------------------------

        telemetry.addData("Program", "Complete");
        telemetry.update();

        visionPortalBack.close();
    } /* runOpMode() */

    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous( int spikemark ) {
        double pos_y=0, pos_x=0, pos_angle=90.0;
        int backdropAprilTagID = -1; // default to RED CENTER

        // We must start in the correct starting orientation for X and Y to be correct
        // Turn toward the backdrop from the RED side
        driveToPosition( 0.0, 0.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO ); // RED
      //driveToPosition( 0.0, 0.0, -90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO ); // BLUE

        // Drive toward backdrop in preparation to score the yellow pixel
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "AprilTag final alignment");
            telemetry.update();
            // Does the camera see ANY of the Backdrop AprilTags?
            boolean targetVisible = updateBackdropAprilTags();
            if( targetVisible ) {
                // Wherever we start, move to where we're aligned to 0 angle error, and 15" away
                // (if we're too far away, or the angle is too large, the AprilTag reading won't be accurate)
                computeBackdropLocation( 15.0, 0.0 );
                pos_y = autoYpos;
                pos_x = autoXpos;
                pos_angle = autoAngle;
                driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                targetVisible = updateBackdropAprilTags();
                if( targetVisible ) {
                    // Move to final position (camera 4" away from the backdrop, 1.5" to the left of CENTER apriltag)
                    computeBackdropLocation( 4.0, -1.5 );
                    pos_y = autoYpos;
                    pos_x = autoXpos;
                    pos_angle = autoAngle;
                    driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
                }
            }
            else {
                telemetry.addData("ERROR", "AprilTag %d not detected", backdropAprilTagID );
                telemetry.update();
            } // !targetVisible (just use odometry)
        } // opModeIsActive

        // Score the yellow pixel
        if( opModeIsActive() ) {
            double desiredDistanceCM;
            double currentDistanceCM;
            double driveOffsetInches;
            telemetry.addData("Motion", "move to backdrop");
            telemetry.update();
            switch( spikemark ) {
                case 1 : desiredDistanceCM = 14.0; break; // LEFT
                case 2:  desiredDistanceCM = 14.0; break; // CENTER
                case 3:
                default: desiredDistanceCM = 14.0; break; // RIGHT
            } // switch
            currentDistanceCM = 6; // robot.getBackdropRange();
            driveOffsetInches = (desiredDistanceCM -currentDistanceCM)/2.54;
//          telemetry.addData("Backdrop Range", "%.1f CM", currentDistanceCM);
//          telemetry.addData("Drive Offset", "%.1f IN", driveOffsetInches);
//          telemetry.update();
//          sleep(3000);
            if( Math.abs(driveOffsetInches) < 7.0 ) {
                pos_x += driveOffsetInches;
                driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO);
            }
            scoreYellowPixel();
        }

    } // mainAutonomous

    /*  HOW TO PARK IN CORNER
    driveToPosition( -15.0, -26.0, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
    driveToPosition( -16.6, -24.1, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
    driveToPosition(  -5.1, -32.1, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU);
    driveToPosition(  -4.0, -37.8, 90.0, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_TO);
    */

} /* AutonomousRescore */
