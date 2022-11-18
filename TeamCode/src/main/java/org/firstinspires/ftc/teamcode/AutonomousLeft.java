/* FTC Team 7572 - Version 1.0 (11/04/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.lang.Math;

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
@Autonomous(name="Autonomous Left", group="7592", preselectTeleOp = "Teleop")
//@Disabled
public class AutonomousLeft extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

//    double    sonarRangeL=0.0, sonarRangeR=0.0, sonarRangeF=0.0, sonarRangeB=0.0;

    OpenCvCamera webcam;
    public int signalZone = 0;   // dynamic (gets updated every cycle during INIT)

    ElapsedTime releaseTimer = new ElapsedTime();

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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(new PipelinePowerPlay(true, true));
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
               // This will be called if the camera could not be opened
            }
        });

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for team color/number
        while (!isStarted()) {
            telemetry.addData("STARTING", "%s", "LEFT");
            telemetry.addData("Signal Detect", "R: " + PipelinePowerPlay.avgR + " G: " +
                    PipelinePowerPlay.avgG + " B: " + PipelinePowerPlay.avgB + " Zone: " +
                    PipelinePowerPlay.signalZone);
            telemetry.update();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Sampling is completed during the INIT stage; No longer need camera active/streaming
        webcam.stopStreaming();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
            signalZone = PipelinePowerPlay.signalZone;
            PipelinePowerPlay.saveLastAutoImage();
        }

        webcam.closeCameraDevice();

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
        driveToPosition( 12.0, 0.0, 0.0, DRIVE_SPEED_50, TURN_SPEED_40 );
        // Strafe right 12"
        driveToPosition( 12.0, 12.0, 0.0, DRIVE_SPEED_50, TURN_SPEED_40 );
        // Turn 180 deg
        driveToPosition( 12.0, 12.0, 179.9, DRIVE_SPEED_50, TURN_SPEED_40 );
        // Stop
        robot.driveTrainMotorsZero();
    } // unitTestOdometryDrive

    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

        // Drive forward to the center-line tall junction pole
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "moveToTallJunction");
            telemetry.update();
            moveToTallJunction();
        }

        // Deposit cone on junction
        if( opModeIsActive() ) {
            telemetry.addData("Skill", "scoreCone");
            telemetry.update();
            scoreCone();
        }

        // Park in signal zone
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "signalZoneParking");
            telemetry.update();
            signalZoneParking( signalZone );
        }

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void moveToTallJunction() {

        // Drive away from wall
        timeDriveStraight( DRIVE_SPEED_20, 750 );

        // Tilt grabber down from stored position
        robot.grabberSetTilt( robot.GRABBER_TILT_STORE );

        // Drive a bit further
        timeDriveStraight( DRIVE_SPEED_20, 750 );

        // Raise lift to scoring position
        robot.liftPosInit( robot.LIFT_ANGLE_HIGH );
        while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
            performEveryLoop();
        }

        // Drive all the way to the tall junction pole
        timeDriveStraight( DRIVE_SPEED_20, 3700 );

        // Turn toward pole
        gyroTurn(TURN_SPEED_20, 45.0 );   // Turn right 45 degrees

        // Drive closer to the pole in order to score
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, 1.0, 999.9, DRIVE_TO );

    } // moveToTallJunction

    /*--------------------------------------------------------------------------------------------*/
    private void scoreCone() {

        // Eject the cone
        releaseTimer.reset();
        robot.grabberSpinEject();
        while( opModeIsActive() && (releaseTimer.milliseconds() < 300) ) {
        }
        robot.grabberSpinStop();

        // Back away from the pole
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -2.0, 999.9, DRIVE_TO );

    } // scoreCone

    /*--------------------------------------------------------------------------------------------*/
    /* +---+---H---+     H = Tall/High junction pole                                              */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* |   | S |   |     S = Starting floor tile                                                  */
    /* +---+---+---/                                                                              */
    private void signalZoneParking( int signalZoneLocation ) {

        if( signalZoneLocation == 1 ) {
            // Realign back to 0 degrees
            gyroTurn(TURN_SPEED_20, 0.0 );
            // Lower lift to driving height
            robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
            while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
                performEveryLoop();
            }
            // Strafe left one tile
            timeDriveStrafe( DRIVE_SPEED_30, 1750 );
            // Back away from center line
            timeDriveStraight( -DRIVE_SPEED_20, 2000 );
        } // // signalZoneLocation 1

        else if( signalZoneLocation == 3 ) {
            // Turn fully 90 deg
            gyroTurn(TURN_SPEED_20, 90.0 );
            // Lower lift to driving height
            robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
            while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
                performEveryLoop();
            }
            // Drive forward one tile
            gyroDrive(DRIVE_SPEED_20, DRIVE_Y, 22.0, 999.9, DRIVE_TO );
            gyroTurn(TURN_SPEED_20, 180.0 );
            gyroDrive(DRIVE_SPEED_20, DRIVE_Y, 10.0, 999.9, DRIVE_TO );
        } // signalZoneLocation 3

        else { // signalZoneLocation 2
            // Realign back to 0 degrees
            gyroTurn(TURN_SPEED_20, 0.0 );
            // Lower lift to driving height
            robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
            // Drive back one tile closer to the cone depot
            gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -20.0, 999.9, DRIVE_TO );
            // Ensure lift has finished the automatic movement
            while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
                performEveryLoop();
            }
        } // signalZoneLocation

    } // signalZoneParking

} /* AutonomousLeft */
