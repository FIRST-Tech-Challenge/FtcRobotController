/* FTC Team 7572 - Version 1.0 (01/26/2023)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/* This program is for a damaged robot.  All it does is inspect the beacon sleeve and park. */
@Autonomous(name="Autonomous _ Right PARK ONLY", group="7592", preselectTeleOp = "Teleop-Right")
@Disabled
public class AutonomousRightP extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    boolean lowCameraInitialized = false;
    boolean backCameraInitialized = false;
    boolean frontCameraInitialized = false;

    OpenCvCamera webcamLow;
    OpenCvCamera webcamFront;
    OpenCvCamera webcamBack;
    public int signalZone = 0;   // dynamic (gets updated every cycle during INIT)

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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

                // This will be called if the camera could not be opened

        webcamBack = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Back"), viewportContainerIds[0]);
        webcamBack.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineBack = new PowerPlaySuperPipeline(false, true, false, false, 144.0);
                webcamBack.setPipeline(pipelineBack);
                webcamBack.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                backCameraInitialized = true;
            }

            @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });
        webcamBack.showFpsMeterOnViewport(false);

        webcamLow = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Low"), viewportContainerIds[1]);
        webcamLow.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineLow = new PowerPlaySuperPipeline(true, false,
                        false, false, 160.0);
                webcamLow.setPipeline(pipelineLow);
                webcamLow.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                lowCameraInitialized = true;
            }

            @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });
        webcamLow.showFpsMeterOnViewport(false);

        while(!(lowCameraInitialized && backCameraInitialized)) {
            sleep(100);
        }
        telemetry.addData("State", "Webcam Initialized");
        telemetry.update();
        pipelineLow.overrideSide(false);  // RIGHT

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        while (!isStarted()) {
            telemetry.addData("ALLIANCE", "%s %c (X=blue O=red)",
                    ((blueAlliance)? "BLUE":"RED"), ((forceAlliance)? '*':' '));
            // If vision pipeline diagrees with forced alliance setting, report it
            if( forceAlliance && (blueAlliance != pipelineLow.isBlueAlliance) )
               telemetry.addData("WARNING!!", "vision pipeline thinks %s !!!", (pipelineLow.isBlueAlliance)? "BLUE":"RED");
            telemetry.addData("STARTING", "%s", "RIGHT");
            telemetry.addData("Signal Detect", "R: " + pipelineLow.avgRR + " G: " +
                    pipelineLow.avgGR + " B: " + pipelineLow.avgBR + " Zone: " +
                    pipelineLow.signalZoneR);
            telemetry.update();
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Force RED alliance?
            if( gamepad1_circle_now && !gamepad1_circle_last ) {
                blueAlliance = false;  // gamepad circle is colored RED
                forceAlliance = true;
            }
            // Force BLUE alliance?
            else if( gamepad1_cross_now && !gamepad1_cross_last ) {
                blueAlliance = true;   // gamepad cross is colored BLUE
                forceAlliance = true;
            }
            // Accept what the vision pipeline detects? (changes real-time!)
            if( !forceAlliance ) {
                blueAlliance = pipelineLow.isBlueAlliance;
            }
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Ensure any movement during robot setup is reset to zero
        globalCoordinatePositionReset();
        
        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
            createAutoStorageFolder(blueAlliance, false);
            pipelineLow.setStorageFolder(storageDir);
            signalZone = pipelineLow.signalZoneR;
            pipelineLow.saveSignalAutoImage( );
        }
        // Turn off detecting the signal.
        pipelineLow.signalDetection(false);

        //---------------------------------------------------------------------------------
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
        // Comment it out if running one of the unit tests above.
        mainAutonomous();
        //---------------------------------------------------------------------------------

        telemetry.addData("Program", "Complete");
        telemetry.update();
    } /* runOpMode() */

    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

        // Park in signal zone
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "signalZoneParking");
            telemetry.update();
            signalZoneParking( signalZone );
        }

    } // mainAutonomous


    /*--------------------------------------------------------------------------------------------*/
    /* +---+---+---+                                                                              */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* |   | S |   |     S = Starting floor tile                                                  */
    /* \---+---+---+                                                                              */
    private void signalZoneParking( int signalZoneLocation ) {

        // We're not moving the lift.  Keep grabber in the stored/init position
        robot.grabberSetTilt( robot.GRABBER_TILT_INIT );

        // Initial movement is just to steer clear of the ground junction in front of the robot
        autoYpos=6.0;  autoXpos=-4.0;  autoAngle=0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_THRU );

        // Now that we're away from the back wall and the ground junction, rotate and move out
        autoYpos=29.0;  autoXpos=-8.0;  autoAngle=+178.0;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_THRU );

        if( signalZoneLocation == 1 ) {  // RED
            // Strafe one tile to the left
            autoXpos=-29.0;
            driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_THRU );
        } // signalZoneLocation 1
        else if( signalZoneLocation == 3 ) {  // BLUE
            // Strafe one tile to the right
            autoXpos=20.0;
            driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_THRU );
        } // signalZoneLocation 3
        else { // signalZoneLocation 2  // GREEN
            autoXpos=-4.0;
            // This can be part of the final movement
        } // signalZoneLocation 2

        // Drive further to center on the group of two tiles
        autoYpos=34.0;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_TO );

    } // signalZoneParking

} /* AutonomousRightP */
