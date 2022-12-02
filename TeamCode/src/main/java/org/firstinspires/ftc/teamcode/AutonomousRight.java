/* FTC Team 7572 - Version 1.1 (11/26/2022)
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
@Autonomous(name="Autonomous Right", group="7592", preselectTeleOp = "Teleop-Right")
//@Disabled
public class AutonomousRight extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

//  double    sonarRangeL=0.0, sonarRangeR=0.0, sonarRangeF=0.0, sonarRangeB=0.0;

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
            telemetry.addData("STARTING", "%s", "RIGHT");
            telemetry.addData("Signal Detect", "R: " + PipelinePowerPlay.avgR + " G: " +
                    PipelinePowerPlay.avgG + " B: " + PipelinePowerPlay.avgB + " Zone: " +
                    PipelinePowerPlay.signalZone);
            telemetry.addData("5-stack cycles", "%d", fiveStackCycles );
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

        if( fiveStackCycles < 1) {
            // Park immediately in signal zone
            if( opModeIsActive() ) {
                telemetry.addData("Motion", "signalZoneParking0");
                telemetry.update();
                signalZoneParking0( signalZone );
            }
        }

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void moveToTallJunction() {

        // Tilt grabber down from stored position
        robot.grabberSetTilt( robot.GRABBER_TILT_STORE );

        // Drive away from wall at slow speed to avoid mecanum roller slippage
        driveToPosition( 4.0, 0.0, 0.0, DRIVE_SPEED_30, TURN_SPEED_40 );

        // Center turret and raise lift to scoring position
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        robot.liftPosInit( robot.LIFT_ANGLE_HIGH );

        // Drive a bit further, but pick up speed
        driveToPosition( 8.0, 0.0, 0.0, DRIVE_SPEED_55, TURN_SPEED_40 );

        // Loop until both motions are complete (robot still moving?)
        while( opModeIsActive() && ((robot.turretMotorAuto == true) || (robot.liftMotorAuto == true)) ) {
            performEveryLoop();
        }

        // Drive all the way to the tall junction pole
        driveToPosition( 49.5, 0.0, 0.0, DRIVE_SPEED_55, TURN_SPEED_40 );
        robot.driveTrainMotorsZero();

        // Turn toward pole
        driveToPosition( 49.5, 0.0, -48.0, DRIVE_SPEED_30, TURN_SPEED_20 );
        robot.driveTrainMotorsZero();

        // Drive closer to the pole in order to score
        driveToPosition( 52.0, -2.5, -48.0, DRIVE_SPEED_30, TURN_SPEED_20 );
        robot.driveTrainMotorsZero();

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
        driveToPosition( 49.5, 0.0, -48.0, DRIVE_SPEED_40, TURN_SPEED_40 );
        robot.driveTrainMotorsZero();

    } // scoreCone

    /*--------------------------------------------------------------------------------------------*/
    /* +---+---H---+     H = Tall/High junction pole                                              */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* |   | S |   |     S = Starting floor tile                                                  */
    /* +---+---+---/                                                                              */
    private void signalZoneParking0( int signalZoneLocation ) {

        if( signalZoneLocation == 1 ) { // RED
            // Turn fully -90 deg
            driveToPosition( 51.0, 0.0, -90.0, DRIVE_SPEED_40, TURN_SPEED_40 );
            robot.driveTrainMotorsZero();
            // Lower lift to driving height
            robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
            while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
                performEveryLoop();
            }
            robot.grabberSetTilt( robot.GRABBER_TILT_INIT );
            // Drive forward one tile
            driveToPosition( 51.0, -24.0, -90.0, DRIVE_SPEED_40, TURN_SPEED_40 );
            driveToPosition( 51.0, -24.0, 180.0, DRIVE_SPEED_40, TURN_SPEED_40 );
            driveToPosition( 42.0, -24.0, 180.0, DRIVE_SPEED_40, TURN_SPEED_40 );
            robot.driveTrainMotorsZero();
        } // signalZoneLocation 1

        else if( signalZoneLocation == 3 ) {  // BLUE
            // Realign back to 0 degrees
            driveToPosition( 51.0, 0.0, 0.0, DRIVE_SPEED_40, TURN_SPEED_40 );
            robot.driveTrainMotorsZero();
            // Lower lift to driving height
            robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
            while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
                performEveryLoop();
            }
            robot.grabberSetTilt( robot.GRABBER_TILT_INIT );
            // Strafe right one tile
            driveToPosition( 51.0, 21.0, 0.0, DRIVE_SPEED_40, TURN_SPEED_40 );
            // Back away from center line
            driveToPosition( 40.0, 21.0, 0.0, DRIVE_SPEED_40, TURN_SPEED_40 );
            robot.driveTrainMotorsZero();
        } // signalZoneLocation 3

        else { // signalZoneLocation 2  // GREEN
            // Realign back to 0 degrees
            driveToPosition( 51.0, 0.0, 0.0, DRIVE_SPEED_40, TURN_SPEED_40 );
            robot.driveTrainMotorsZero();
            // Lower lift to driving height
            robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
            // Ensure lift has finished the automatic movement
            while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
                performEveryLoop();
            }
            robot.grabberSetTilt( robot.GRABBER_TILT_INIT );
            // Drive back one tile closer to the cone depot
            driveToPosition( 32.0, 0.0, 0.0, DRIVE_SPEED_40, TURN_SPEED_40 );
            robot.driveTrainMotorsZero();
        } // signalZoneLocation

    } // signalZoneParking0

} /* AutonomousRight */
