/* FTC Team 7572 - Version 1.2 (12/15/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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
@Autonomous(name="Autonomous Left", group="7592", preselectTeleOp = "Teleop-Left")
//@Disabled
public class AutonomousLeft extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)
    boolean lowCameraInitialized = false;
    boolean backCameraInitialized = false;
    boolean frontCameraInitialized = false;

//  double    sonarRangeL=0.0, sonarRangeR=0.0, sonarRangeF=0.0, sonarRangeB=0.0;

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

        // To add front camera back, need to update containers to 3.
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

        webcamBack = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Back"), viewportContainerIds[0]);
        webcamBack.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineBack = new PowerPlaySuperPipeline(false, true,
                        false, false, 160.0, blueAlliance, false);
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
                        !blueAlliance, blueAlliance, 160.0, blueAlliance, true);
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

        /*
        webcamFront = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Front"), viewportContainerIds[2]);
        webcamFront.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineFront = new PowerPlaySuperPipeline(false, true,
                        false, false, 160.0, blueAlliance, false);
                webcamFront.setPipeline(pipelineFront);
                webcamFront.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                frontCameraInitialized = true;
            }

            @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });
        webcamFront.showFpsMeterOnViewport(false);
         */

        // To add front camera, need to add checking for frontCameraInitialized.
        while(!(lowCameraInitialized && backCameraInitialized)) {
            sleep(100);
        }
        telemetry.addData("State", "Webcam Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        while (!isStarted()) {
            telemetry.addData("ALLIANCE", "%s (%s)", (blueAlliance)? "BLUE":"RED", "X=blue O=red");
            telemetry.addData("STARTING", "%s", "LEFT");
            telemetry.addData("Signal Detect", "R: " + pipelineLow.avgRL + " G: " +
                    pipelineLow.avgGL + " B: " + pipelineLow.avgBL + " Zone: " +
                    pipelineLow.signalZoneL);
            telemetry.addData("5-stack cycles", "%d", fiveStackCycles );
            telemetry.addData("(use %s bumpers to modify", "LEFT/RIGHT");
            telemetry.update();
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Change to RED/BLUE alliance?
            if( gamepad1_circle_now && !gamepad1_circle_last ) {
                blueAlliance = false;  // gamepad circle is colored RED
            }
            else if( gamepad1_cross_now && !gamepad1_cross_last ) {
                blueAlliance = true;   // gamepad cross is colored BLUE
            }            
            // Change number of 5-stack to attempt?
            if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last ) {
              fiveStackCycles -= 1;
              if( fiveStackCycles < 0 ) fiveStackCycles=0;              
            }
            else if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last ) {
              fiveStackCycles += 1;
              if( fiveStackCycles > 5 ) fiveStackCycles=5;              
            }            
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Start the autonomous timer so we know how much time is remaining for cone cycling
        autonomousTimer.reset();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
            signalZone = pipelineLow.signalZoneL;
            pipelineLow.saveLastAutoImage();
        }
        // Turn off detecting the signal.
        pipelineLow.signalDetection(false);
        // Enable objectdetection of objects we are interested in
        if(blueAlliance) {
            pipelineLow.blueConeDetection(true);
        } else {
            pipelineLow.redConeDetection(true);
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
    private void mainAutonomous() {

        // Drive forward to the center-line tall junction pole
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "moveToTallJunction");
            telemetry.update();
            moveToTallJunction();
        }

        // Center on pole
//        if( opModeIsActive()) {
//            telemetry.addData("Skill", "rotateToCenterPole");
//            telemetry.update();
//            rotateToCenterPole();
//        }

        // Adjust distance to pole
//        if( opModeIsActive()) {
//            telemetry.addData("Skill", "distanceFromFront");
//            telemetry.update();
//            distanceFromFront(28.0, 1.0);
//        }

        // Deposit cone on junction
        if( opModeIsActive() ) {
            telemetry.addData("Skill", "scoreCone");
            telemetry.update();
            scoreCone();
        }

        // Lets cycle:
        // Step 1. Rotate towards stack and get a decent starting position
        // Step 2. Align to cone (red/blue specific logic)
        // Step 3. Range from cone
        // Step 4. Collect cone, custom heights
        // Step 5. Rotate towards pole and get a decent starting position
        // Step 6. Score cone
        // Step 7. Profit
        double cycleDistance = 28.0;
        fiveStackCycles = 1;    // FORCE TO 1 FOR TOURNY4 (see default in AutonomousBase)`
        while (opModeIsActive() && (autonomousTimer.milliseconds() < 16000) && (fiveStackCycles > 0)) {
            if (opModeIsActive()) {
                telemetry.addData("Skill", "moveToConeStack");
                telemetry.update();
                moveToConeStack();
            }

            if (opModeIsActive()) {
                telemetry.addData("Skill", "rotateToConeStack");
                telemetry.update();
                if( blueAlliance )
                  rotateToCenterBlueCone();
                else
                  rotateToCenterRedCone();
            }

            if( opModeIsActive()) {
                switch(fiveStackHeight) {
                    case 5:  cycleDistance = 30.0; break;
                    case 4:  cycleDistance = 30.0; break;
                    case 3:  cycleDistance = 30.0; break;
                    default: cycleDistance = 30.0;
                }
                telemetry.addData("Skill", "distanceToConeStack");
                telemetry.update();
                distanceFromFront(cycleDistance, 1.0);
            }

            if (opModeIsActive()) {
                telemetry.addData("Skill", "collectCone");
                telemetry.update();
                collectCone();  // decrements fiveStackHeight!
            }

            if (opModeIsActive()) {
                telemetry.addData("Skill", "moveToTallJunctionFromStack");
                telemetry.update();
                moveToTallJunctionFromStack();
            }

            if( opModeIsActive()) {
                telemetry.addData("Skill", "rotateToCenterPole");
                telemetry.update();
                rotateToCenterPole();
            }

            if( opModeIsActive()) {
                telemetry.addData("Skill", "distanceFromFront");
                telemetry.update();
                distanceFromFront(35.0, 1.0);
            }

            if( opModeIsActive() ) {
                telemetry.addData("Skill", "scoreStackCone");
                telemetry.update();
                scoreCone();
            }

            fiveStackCycles--;
        } // while()

        // Park in signal zone
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "signalZoneParking");
            telemetry.update();
            signalZoneParking( signalZone );
        }
//        while(opModeIsActive()) { sleep(100); }
    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void moveToTallJunction() {

        // Tilt grabber down from autonomous starting position (vertical)
        // so we're clear to raise the lift and not hit the front lift motor
        // (since we're turning outward toward GROUND junction it's okay to exceed 24" tile width
        robot.grabberSetTilt( robot.GRABBER_TILT_STORE );

        // Initial movement accomplishes two goals:
        // 1. Avoid the ground junction in front of the robot (5.5" rightward shift)
        // 2. Turn 90deg  so we don't entrap the beacon cone
        autoYpos=18.0;  autoXpos=5.5;  autoAngle=-90.0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_50, TURN_SPEED_60, DRIVE_THRU );

        // The grabber finished the tilt down during the 90deg turn movement, so
        // it's safe now to command the lift to raise to scoring position
        robot.liftPosInit( robot.LIFT_ANGLE_HIGH_B );

        // We're past the medium junction pole, so okay to rotate the turret
        robot.turretPosInit( -32.5 );

        // Drive partway there (while lift raises past the front motor)
        autoYpos=34.5;  autoXpos=4.0;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_100, TURN_SPEED_80, DRIVE_THRU );

        // Tilt grabber backward to final scoring position and rotate cone over
        robot.grabberSetTilt( robot.GRABBER_TILT_BACK_H );
        robot.rotateServo.setPosition( robot.GRABBER_ROTATE_DOWN );

        // Drive the final distance to the high junction pole
        autoYpos=54.0;  autoXpos=8.0;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_100, TURN_SPEED_80, DRIVE_TO );

        // Both mechanisms should be finished, but pause here if they haven't (until they do)
        while( opModeIsActive() && ((robot.turretMotorAuto == true) || (robot.liftMotorAuto == true)) ) {
            performEveryLoop();
        }

    } // moveToTallJunction

    /*--------------------------------------------------------------------------------------------*/
    private void scoreCone() {

        // Eject the cone
        intakeTimer.reset();
        robot.grabberSpinEject();
        // Wait 300 msec
        while( opModeIsActive() && (intakeTimer.milliseconds() < 300 || robot.bottomConeSensor.getState()) ) {
            performEveryLoop();
        }
        // Stop the ejector
        robot.grabberSpinStop();
        robot.grabberSetTilt( robot.GRABBER_TILT_STORE );

    } // scoreCone

    /*--------------------------------------------------------------------------------------------*/
    private void moveToConeStack() {

        // Establish targets for turret angle (centered) and lift height (5-stack)
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        robot.liftPosInit( robot.LIFT_ANGLE_5STACK );

        // Having just scored on the tall poll, turn left (-90deg) to point toward the 5-stack
        autoYpos=50.0;  autoXpos=-7.0;  autoAngle=-90.0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_TO );

        while( opModeIsActive() && ((robot.turretMotorAuto == true) || (robot.liftMotorAuto == true)) ) {
            performEveryLoop();
        }
        sleep(30000);

        // Drive closer to the 5-stack against the wall (same Y and ANGLE, but new X)
        autoXpos=-16.0;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );
        while( opModeIsActive() && ((robot.turretMotorAuto == true) || (robot.liftMotorAuto == true)) ) {
            performEveryLoop();
        }

    } // moveToConeStack

    /*--------------------------------------------------------------------------------------------*/
    // Assumes we've already completed the alignment on the 5-stack (rotateToCenterBlueCone or
    // rotateToCenterRedCone and distanceFromFront so we're ready to actually collect the cone
    private void collectCone() {
        double liftAngle5stack;

        // Lower the collector to the horizontal collecting position
        robot.grabberSetTilt( robot.GRABBER_TILT_GRAB );

        // Determine the correct lift-angle height based on how many cones remain
        // 80.6 height to light cone to after collecting, and for sonar
        // Range 28, 28, 29
        // + 14.8
        switch( fiveStackHeight ) {
            case 5  : liftAngle5stack = 109.1; break;
            case 4  : liftAngle5stack =  97.9; break;
            case 3  : liftAngle5stack = 101.7; break;
            case 2  : liftAngle5stack = 105.0; break; // TODO: Not measured
            case 1  : liftAngle5stack = 110.0; break; // TODO: Not measured
            default : liftAngle5stack = 94.3;
        } // switch()

        // Lower the lift to the desired height (and ensure we're centered)
        robot.liftPosInit( liftAngle5stack );
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        while( opModeIsActive() && ((robot.turretMotorAuto == true) || (robot.liftMotorAuto == true)) ) {
            performEveryLoop();
        }

        // Start the collector spinning
        robot.grabberSpinCollect();
        intakeTimer.reset();
        // start to slowly lower onto cone
        robot.liftMotorsSetPower( -0.20 );
        while(robot.topConeSensor.getState() && intakeTimer.milliseconds() <= 600) {
            performEveryLoop();
        }
        // stop the collector
        robot.grabberSpinStop();
        // reverse the lift to raise off the cone stack
        robot.liftPosInit( robot.LIFT_ANGLE_5STACK );
        while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
            performEveryLoop();
        }
//        robot.liftMotorsSetPower( 0.40 );
//        sleep( 1500 );  // 1.5 sec
        // halt lift motors
        robot.liftMotorsSetPower( 0.0 );

        // Reduce the remaining cone-count
        fiveStackHeight--;
    } // collectCone

    /*--------------------------------------------------------------------------------------------*/
    private void moveToTallJunctionFromStack() {

        // Perform setup to center turret and raise lift to scoring position
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        robot.liftPosInit( robot.LIFT_ANGLE_AUTO_H );
        robot.grabberSetTilt( robot.GRABBER_TILT_AUTO_F );

        // Drive back to tall junction (adjusting lift along the way)
        autoYpos=51.0;  autoXpos=0.0;  autoAngle=35.0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );

        // Re-center turret again (if it shifted while driving)
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        while( opModeIsActive() && (robot.turretMotorAuto == true) ) {
            performEveryLoop();
        }

    } // moveToTallJunctionFromStack

    /*--------------------------------------------------------------------------------------------*/
    /* +---+---H---+     H = Tall/High junction pole on RIGHT                                     */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* |   | S |   |     S = Starting floor tile                                                  */
    /* +---+---+---/                                                                              */
    private void signalZoneParking( int signalZoneLocation ) {

        // TODO: This code assumes autoYpos, autoXpos, autoAngle carry over from
        // scoring on the tall poll.  If that changes (ie, we go for a different pole,
        // or don't complete that operation, then autoYpos and autoXpos will need to
        // be redefined here to the correct values.

        switch( signalZoneLocation ) {
           case 3  : autoAngle=+90.0; break; // Turn fully to +90deg (BLUE)
           default : autoAngle=  0.0; break; // Realign back to 0deg (RED/GREEN)
        }
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );

        // Now that we've turned away from the pole, lower lift to driving position
        robot.grabberSetTilt( robot.GRABBER_TILT_SAFE );
        robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );

        if( signalZoneLocation == 1 ) {  // RED
           // Strafe left one tile
           autoXpos -= 21.0;
           driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_THRU );
           // Back away from center line, but stay within Signal Zone 1
           autoYpos -= 11.0;
           driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );
        } // // signalZoneLocation 1
        else if( signalZoneLocation == 3 ) {  // BLUE
           // Drive forward one tile pointing 90deg
           autoXpos += 24.0;
           driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_THRU );
           // Turn back toward substation
           autoAngle = 180.0;
           driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_THRU );
           // Drive closer to the substation to center in Signal Zone 3
           autoYpos -= 9.0;
           driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );
        } // signalZoneLocation 3
        else { // signalZoneLocation 2  // GREEN
           // Drive back one tile closer to the substation in Signal Zone 2
           autoYpos -= 19.0;
           driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );
        } // signalZoneLocation 2

        // Ensure we complete all lift movement before ending autonomous
        while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
            performEveryLoop();
        }

        // Raise collector straight up (prevents "droop" when power is removed)
        robot.grabberSetTilt( robot.GRABBER_TILT_INIT );

    } // signalZoneParking

} /* AutonomousLeft */
