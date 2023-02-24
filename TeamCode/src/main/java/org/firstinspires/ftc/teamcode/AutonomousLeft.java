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

        if(alignToFront) {
            webcamFront = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                    "Webcam Front"), viewportContainerIds[0]);
            webcamFront.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    pipelineFront = new PowerPlaySuperPipeline(false, true, false, false, 176.0);
                    webcamFront.setPipeline(pipelineFront);
                    webcamFront.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    frontCameraInitialized = true;
                }

                @Override
                public void onError(int errorCode) {
                    // This will be called if the camera could not be opened
                }
            });
            webcamFront.showFpsMeterOnViewport(false);
        } else {
            webcamBack = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                    "Webcam Back"), viewportContainerIds[0]);
            webcamBack.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    pipelineBack = new PowerPlaySuperPipeline(false, true, false, false, 144.0);
                    webcamBack.setPipeline(pipelineBack);
                    webcamBack.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    backCameraInitialized = true;
                }

                @Override
                public void onError(int errorCode) {
                    // This will be called if the camera could not be opened
                }
            });
            webcamBack.showFpsMeterOnViewport(false);
        }

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

        while(!(lowCameraInitialized && (backCameraInitialized || frontCameraInitialized))) {
            sleep(100);
        }
        telemetry.addData("State", "Webcam Initialized");
        telemetry.update();
        pipelineLow.overrideSide(true);  // LEFT

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        while (!isStarted()) {
            telemetry.addData("ALLIANCE", "%s %c (X=blue O=red)",
                    ((blueAlliance)? "BLUE":"RED"), ((forceAlliance)? '*':' '));
            // If vision pipeline diagrees with forced alliance setting, report it
            if( forceAlliance && (blueAlliance != pipelineLow.isBlueAlliance) )
               telemetry.addData("WARNING!!", "vision pipeline thinks %s !!!", (pipelineLow.isBlueAlliance)? "BLUE":"RED");
            telemetry.addData("STARTING", "%s", "LEFT");
            telemetry.addData("Signal Detect", "R: " + pipelineLow.avgRL + " G: " +
                    pipelineLow.avgGL + " B: " + pipelineLow.avgBL + " Zone: " +
                    pipelineLow.signalZoneL);
            telemetry.addData("5-stack cycles", "%d", fiveStackCycles );
            telemetry.addData("","(use %s bumpers to modify", "LEFT/RIGHT");
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
            createAutoStorageFolder(blueAlliance, true);
            pipelineLow.setStorageFolder(storageDir);
            if(alignToFront) {
                pipelineFront.setStorageFolder(storageDir);
            } else {
                pipelineBack.setStorageFolder(storageDir);
            }
            signalZone = pipelineLow.signalZoneL;
            pipelineLow.overrideAlliance(blueAlliance);
            pipelineLow.saveLastAutoImage( );
        }
        // Turn off detecting the signal.
        pipelineLow.signalDetection(false);
        // Enable object detection of objects we are interested in
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
        double[] timePoleArrive  = {0,0,0,0,0,0};
        double[] timePoleDepart  = {0,0,0,0,0,0};
        double[] timeStackArrive = {0,0,0,0,0,0};
        double[] timeStackDepart = {0,0,0,0,0,0};
        double   timeNow;
        int      timeIndex = 0;

        // Drive forward to the center-line tall junction pole
        if( opModeIsActive() ) {
            timeNow = autonomousTimer.milliseconds()/1000.0;
            telemetry.addData("Motion", "moveToTallJunction (%.1f)", timeNow );
            telemetry.update();
            moveToTallJunction();
            timeNow = autonomousTimer.milliseconds()/1000.0;
            timePoleArrive[timeIndex] = timeNow;
        }

        // Center on pole
      if( opModeIsActive()) {
         timeNow = autonomousTimer.milliseconds()/1000.0;
          telemetry.addData("Skill", "alignToPole (%.1f)", timeNow );
          telemetry.update();
          alignToPole(true);
      }

        // Deposit cone on junction
        if( opModeIsActive() ) {
            timeNow = autonomousTimer.milliseconds()/1000.0;
            telemetry.addData("Skill", "scoreCone (%.1f)", timeNow );
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
        int cycleDistance;
        double newCycleTimeout  = 21500.0;  // 21.5 sec elapsed (don't start another when less than 8.5 sec left)
        double poleAlignTimeout = 26000.0;  // 26.0 sec elapsed (
        while (opModeIsActive() && (autonomousTimer.milliseconds() <= newCycleTimeout) && (fiveStackCycles > 0)) {
            if (opModeIsActive()) {
                timeNow = autonomousTimer.milliseconds()/1000.0;
                telemetry.addData("Skill", "moveToConeStack (%.1f)", timeNow );
                telemetry.update();
                timePoleDepart[timeIndex] = timeNow;
                moveToConeStack();
            }

            if (opModeIsActive()) {
                switch(fiveStackHeight) {
                    case 5:  cycleDistance = 28; break;
                    case 4:  cycleDistance = 28; break;
                    case 3:  cycleDistance = 28; break;
                    case 2:  cycleDistance = 27; break;
                    case 1:  cycleDistance = 27; break;
                    default: cycleDistance = 27;
                }
                timeNow = autonomousTimer.milliseconds()/1000.0;
                telemetry.addData("Skill", "alignToConeStack (%.1f)", timeNow );
                telemetry.update();
                timeStackArrive[timeIndex] = timeNow;
                alignToConeStack(blueAlliance, cycleDistance);
            }

            if (opModeIsActive()) {
                timeNow = autonomousTimer.milliseconds()/1000.0;
                telemetry.addData("Skill", "collectCone (%.1f)", timeNow );
                telemetry.update();
                collectCone();  // decrements fiveStackHeight!
            }

            if (opModeIsActive()) {
                timeNow = autonomousTimer.milliseconds()/1000.0;
                telemetry.addData("Skill", "moveToTallJunctionFromStack (%.1f)", timeNow );
                telemetry.update();
                timeStackDepart[timeIndex] = timeNow;
                moveToTallJunctionFromStack();
            }

            if( opModeIsActive()) {
                timeNow = autonomousTimer.milliseconds()/1000.0;
                telemetry.addData("Skill", "alignToPole (%.1f)", timeNow );
                telemetry.update();
                // make sure we have time left to alignToPole and then park!
                // (otherwise just drop it and park)
                if( autonomousTimer.milliseconds() <= poleAlignTimeout ) {
                    alignToPole(true);
                }
            }

            if( opModeIsActive() ) {
                timeNow = autonomousTimer.milliseconds()/1000.0;
                telemetry.addData("Skill", "scoreCone (%.1f)", timeNow );
                telemetry.update();
                scoreCone();
            }

            fiveStackCycles--;
        } // while()  ------------------------------------------------------------

        // Park in signal zone
        if( opModeIsActive() ) {
            timeNow = autonomousTimer.milliseconds()/1000.0;
            telemetry.addData("Motion", "signalZoneParking (%.1f)", timeNow );
            telemetry.update();
            signalZoneParking( signalZone );
        }

        // Ensure both lift and turret are stopped
        robot.liftMotorsSetPower( 0.0 );
        robot.turretMotor.setPower( 0.0 );

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void moveToTallJunction() {

        // Tilt grabber down from autonomous starting position (vertical) so we're clear
        // to raise the lift and not hit the front lift motor (since we're turning outward
        // toward GROUND junction, it's okay for robot length to exceed the 24" tile width
        robot.grabberSetTilt( robot.GRABBER_TILT_STORE );
//        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H );
//        robot.liftPIDPosInit( robot.LIFT_ANGLE_HIGH_A);
//        robot.turretPIDPosInit( robot.TURRET_ANGLE_AUTO_R );

        // Initial movement is just to steer clear of the ground junction in front of the robot
        // (once we start to rotate the robot becomes much wider)
        autoYpos=6.0;  autoXpos=4.0;  autoAngle=0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_60, TURN_SPEED_60, DRIVE_THRU );

        // The 2nd movement is to rotate 90deg so we don't entrap the beacon cone
        robot.liftPIDPosInit( robot.LIFT_ANGLE_HIGH_A);
        robot.turretPIDPosInit( robot.TURRET_ANGLE_AUTO_R );
        autoYpos=18.0;  autoXpos=5.5;  autoAngle=-90.0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_60, TURN_SPEED_60, DRIVE_THRU );

        // Drive partway there very fast
        autoYpos=34.5;  autoXpos=4.5;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_100, TURN_SPEED_80, DRIVE_THRU );

        // Drive the final distance to the high junction pole
        autoYpos=55.3;  autoXpos=7.0;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_70, DRIVE_TO );

        // Both mechanisms should be finished, but pause here if they haven't (until they do)
        while( opModeIsActive() && ((robot.turretMotorPIDAuto == true) || (robot.liftMotorPIDAuto == true)) ) {
            performEveryLoop();
        }

        // Tilt grabber down to final scoring position
        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H );
        sleep( 500 );

    } // moveToTallJunction

    /*--------------------------------------------------------------------------------------------*/
    private void scoreCone() {

        // Start ejecting the cone
        intakeTimer.reset();
        robot.grabberSpinEject();
        // Wait for sensor to indicate it's clear (or we timeout)
        while( opModeIsActive() ) {
            performEveryLoop();
            // Ensure we eject for at least 250 msec before using sensor (in case sensor fails)
            boolean bottomSensorClear = robot.bottomConeState && (intakeTimer.milliseconds() > 250);
            // Also have a max timeout in case sensor fails
            boolean maxEjectTimeReached = (intakeTimer.milliseconds() >= 400);
            // Is cycle complete?
            if( bottomSensorClear || maxEjectTimeReached) break;
        }
        // Stop the ejector
        robot.grabberSpinStop();
        robot.grabberSetTilt( robot.GRABBER_TILT_STORE );

    } // scoreCone

    /*--------------------------------------------------------------------------------------------*/
    private void moveToConeStack() {

        // Establish targets for turret angle (centered) and lift height (5-stack)
        robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );
        robot.liftPIDPosInit( robot.LIFT_ANGLE_5STACK );

        // Having just scored on the tall poll, turn left (-90deg) to point toward the 5-stack
        autoYpos=51.5;  autoXpos=-7.0;  autoAngle=-90.0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_THRU );
        robot.rotateServo.setPosition( robot.GRABBER_ROTATE_UP );

        // Drive closer to the 5-stack against the wall (same Y and ANGLE, but new X)
        autoXpos=-13.0;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );
        while( opModeIsActive() && ((robot.turretMotorPIDAuto == true) || (robot.liftMotorPIDAuto == true)) ) {
            performEveryLoop();
        }
    } // moveToConeStack

    /*--------------------------------------------------------------------------------------------*/
    // Assumes we've already completed the alignment on the 5-stack (rotateToCenterBlueCone or
    // rotateToCenterRedCone and distanceFromFront so we're ready to actually collect the cone
    private void collectCone() {
        double liftAngle5stack;

        // Lower the collector to the nearly-horizontal collecting position
        robot.grabberSetTilt( robot.GRABBER_TILT_GRAB2 );

        // Determine the correct lift-angle height based on how many cones remain
        switch( fiveStackHeight ) {
            case 5  : liftAngle5stack = 103.0; break;
            case 4  : liftAngle5stack = 106.0; break;
            case 3  : liftAngle5stack = 110.0; break;
            case 2  : liftAngle5stack = 111.0; break; // TODO: Not measured
            case 1  : liftAngle5stack = 111.0; break; // TODO: Not measured
            default : liftAngle5stack = 111.0;
        } // switch()

        // Lower the lift to the desired height (and ensure we're centered)
        robot.liftPIDPosInit( liftAngle5stack );
        robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );
        while( opModeIsActive() && ((robot.turretMotorPIDAuto == true) || (robot.liftMotorPIDAuto == true)) ) {
            performEveryLoop();
        }

        // Start the collector spinning
        robot.grabberSpinCollect();
        intakeTimer.reset();
        // start to slowly lower onto cone
        robot.liftMotorsSetPower( -0.25 );
        while(robot.topConeState && intakeTimer.milliseconds() <= 600) {
            performEveryLoop();
            // Limit DOWNWARD lift movement even if collector is still lifting cone up to sensor
            if( robot.liftAngle >= robot.LIFT_ANGLE_MAX ) {
              robot.liftMotorsSetPower( 0.0 );
              }
        }
        // stop the collector, and halt lift motors (if not already)
        robot.liftMotorsSetPower( 0.0 );
        robot.grabberSpinStop();

        // Raise the collector so we don't clip the wall with the cone on the way up
        robot.grabberSetTilt( robot.GRABBER_TILT_GRAB3 );

        // Now reverse the lift to raise off the cone stack
        robot.liftPIDPosInit( robot.LIFT_ANGLE_5STACK );
        while( opModeIsActive() && (robot.liftMotorPIDAuto == true) ) {
            performEveryLoop();
        }
        // halt lift motors
        robot.liftMotorsSetPower( 0.0 );

        // Reduce the remaining cone-count
        fiveStackHeight--;
    } // collectCone

    /*--------------------------------------------------------------------------------------------*/
    private void moveToTallJunctionFromStack() {

        // Perform setup to center turret and raise lift to scoring position
        robot.turretPIDPosInit( robot.TURRET_ANGLE_5STACK_L);
        robot.liftPIDPosInit( robot.LIFT_ANGLE_HIGH_A);
        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H );

        // Drive back to tall junction (adjusting lift along the way)
        // (stay along Y=51.5 instead of returning to Y=54.0, but rotate turret more (-56.5, not -34.5)
        autoYpos=51.5;  autoXpos=10.0;  autoAngle=-90.0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );

        // Re-center turret again (if it shifted while driving)
        while( opModeIsActive() && (robot.turretMotorPIDAuto == true) ) {
            performEveryLoop();
        }

//========== TEST MODE ========
//      while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
//          performEveryLoop();
//      }
//      sleep( 30000 );
//========== TEST MODE ========

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

        // Tilt the collector up away from the pole we just scored on
        robot.grabberSetTilt( robot.GRABBER_TILT_SAFE );
        robot.rotateServo.setPosition( robot.GRABBER_ROTATE_UP );

        // Initialize so that turret rotates back to center as we turn
        robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );

        // Determine the angle to turn the drivetrain
        switch( signalZoneLocation ) {
            case 2  : autoAngle=-179.9; break; // Turn fully to -180deg (GREEN)
            default : autoAngle=-90.0;  break; // Remain at -90deg (RED/BLUE)
        }
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );

        // Initialize so that lift lowers to driving position
        robot.liftPIDPosInit( robot.LIFT_ANGLE_COLLECT );

        if( signalZoneLocation == 1 ) {  // RED
            // Drive one tile closer to field wall
            autoYpos=51.5;  autoXpos=-8.0;  autoAngle=-90.0;    // (inches, inches, degrees)
            driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_70, TURN_SPEED_60, DRIVE_THRU );
            // Turn back toward substation
            autoXpos=-9.0;  autoAngle = -179.9;
            driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_THRU );
            // Back away from center line, but stay within Signal Zone 1
            autoYpos=38.5;  autoXpos=-17.0;
            robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );
            driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );
        } // signalZoneLocation 1
        else if( signalZoneLocation == 3 ) {  // BLUE
            // Drive forward one tile pointing 90deg
            autoYpos=51.5;  autoXpos=21.5;  autoAngle=-90.0;    // (inches, inches, degrees)
            driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_70, TURN_SPEED_60, DRIVE_THRU );
            // Turn back toward substation
            autoAngle = -179.9;
            driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_THRU );
            // Drive closer to the substation to center in Signal Zone 3
            autoYpos=38.5;  autoXpos=28.0;
            robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );
            driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );
        } // signalZoneLocation 3
        else { // signalZoneLocation 2  // GREEN
            // Drive back one tile closer to the substation in Signal Zone 2
            autoYpos=38.5;  autoXpos=4.0;
            robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );
            driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );
        } // signalZoneLocation 2

        // Ensure we complete all lift movement before ending autonomous
        while( opModeIsActive() && (robot.liftMotorPIDAuto == true) ) {
            performEveryLoop();
        }

        // Raise collector straight up (prevents "droop" when power is removed)
        robot.grabberSetTilt( robot.GRABBER_TILT_INIT );

    } // signalZoneParking

} /* AutonomousLeft */
