/* FTC Team 7572 - Version 1.0 (11/11/2023)
*/
package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name="Autonomous Left-Blue", group="7592", preselectTeleOp = "Teleop-Left")
//@Disabled
public class AutonomousLeftBlue extends AutonomousBase {

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
        // This is left side blue alliance.
        pipelineLeft = new CenterstageSuperPipeline(true, false );
// WAS 640, 480
        visionPortalLeft = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam Left"))
                .addProcessor(pipelineLeft)
                .setCameraResolution(new Size(1280, 720))
                .build();
//        pipelineRight = new CenterstageSuperPipeline(false, false);
//        visionPortalRight = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam Right"))
//                .addProcessor(pipelineRight)
//                .setCameraResolution(new Size(640, 480))
//                .build();
//      pipelineBack = new CenterstageSuperPipeline(true, false);
//        webcamBack.setCamera(hardwareMap.get(WebcamName.class, "Webcam Back"));
//        webcamBack.setCameraResolution(new Size(320, 240));
//        webcamBack.addProcessor(pipelineBack);

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        while (!isStarted()) {
            telemetry.addData("ALLIANCE", "%s %c (X=blue O=red)",
                    ((redAlliance)? "RED":"BLUE"), ((forceAlliance)? '*':' '));
            // If vision pipeline disagrees with forced alliance setting, report it
            if( forceAlliance && (redAlliance != pipelineLeft.redAlliance) )
               telemetry.addData("WARNING!!", "vision pipeline thinks %s !!!", (pipelineLeft.redAlliance)? "RED":"BLUE");
            telemetry.addData("STARTING", "%s", "LEFT");
            telemetry.addData("TeamProp", " Hue("  + pipelineLeft.targetHue +
                                          ") L:"   + pipelineLeft.avg1   +
                                          " C:"    + pipelineLeft.avg2   + 
                                          " R:"    + pipelineLeft.avg3   +
                                          " Zone:" + pipelineLeft.spikeMark );
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
                redAlliance = pipelineLeft.redAlliance;
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
            spikeMark = pipelineLeft.spikeMark;
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
/*    private void updatePoleAlignInstrumentation() {
        // Collect our instrumentation data
        anglePole1[timeIndex]   = robot.turretAngle;
        odomPoleX0[timeIndex]   = beforeXpos;
        odomPoleY0[timeIndex]   = beforeYpos;
        odomPoleAng0[timeIndex] = beforeAngle;
        odomPoleX1[timeIndex]   = afterXpos;
        odomPoleY1[timeIndex]   = afterYpos;
        odomPoleAng1[timeIndex] = afterAngle;
        // Populate the telemetry
        telemetry.addData("Drive", "Pole Travel %.1f sec", timePoleDrive[timeIndex] );
        telemetry.addData("Turret", "Before=%.1f, After=%.1f degrees",
                anglePole0[timeIndex], anglePole1[timeIndex] );
        telemetry.addData("Odometry", "X=%.1f %.1f, Y=%.1f %.1f, Angle=%.1f %.1f",
                odomPoleX0[timeIndex],   odomPoleX1[timeIndex],
                odomPoleY0[timeIndex],   odomPoleY1[timeIndex],
                odomPoleAng0[timeIndex], odomPoleAng1[timeIndex] );
    } // updatePoleAlignInstrumentation
*/
    /*--------------------------------------------------------------------------------------------*/
/*    private void updateStackAlignInstrumentation() {
        // Collect our instrumentation data
        odomStackX0[timeIndex]   = beforeXpos;
        odomStackY0[timeIndex]   = beforeYpos;
        odomStackAng0[timeIndex] = beforeAngle;
        odomStackX1[timeIndex]   = afterXpos;
        odomStackY1[timeIndex]   = afterYpos;
        odomStackAng1[timeIndex] = afterAngle;
        // Populate the telemetry
        telemetry.addData("Drive", "Stack Travel %.1f sec", timeStackDrive[timeIndex] );
        telemetry.addData("Odometry", "X=%.1f %.1f, Y=%.1f %.1f, Angle=%.1f %.1f",
                odomStackX0[timeIndex],   odomStackX1[timeIndex],
                odomStackY0[timeIndex],   odomStackY1[timeIndex],
                odomStackAng0[timeIndex], odomStackAng1[timeIndex] );
    } // updateStackAlignInstrumentation
*/
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous( int spikemark ) {
        double drive_power = 0.6;
        double strafe_power = 0.6;

     // Drive forward to spike mark
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to Spike Mark");
            telemetry.update();
            // All 3 positions required forward movement
//          timeDriveStraight( drive_power, 800 );
            driveToPosition( -12.0, 0.0, 0.0, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_THRU );
            // THe final motion depends on whether it's left/center/right spike (1/2/3)
            switch( spikemark ) {
                case 1 : // LEFT
                    timeDriveStrafe( -strafe_power, 525 );
                    break;
                case 2:  // CENTER
                    timeDriveStraight( drive_power, 200 );
                    break;
                case 3:  // RIGHT
                default:
                    timeDriveStrafe( strafe_power,474 );   // less! (against the truss)
                    break;
            } // switch
        }

        // Eject purple pixel
        if( opModeIsActive()) {
            telemetry.addData("Skill", "eject purple pixel");
            telemetry.update();
            robot.collectorMotor.setPower(robot.COLLECTOR_EJECT_POWER);
            sleep(1600 );  // 1.6 sec
            robot.collectorMotor.setPower(0.0);
        }

        // Park in back stage
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "park in back stage");
            telemetry.update();
            timeDriveStraight( -drive_power, 200 );
            // FOR NOW, CAN'T PARK FROM RIGHT SIDE
        }

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void moveToJunction( boolean highJunction ) {
/*
        // Tilt grabber down from autonomous starting position (vertical) so we're clear
        // to raise the lift and not hit the front lift motor, but keep it mostly verticle
        // for a safe driving configuration (in case we run into something)
        robot.grabberSetTilt( robot.GRABBER_TILT_STORE );

        // Initial movement is just to steer clear of the ground junction in front of the robot
        autoYpos=6.0;  autoXpos=4.0;  autoAngle=0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_60, TURN_SPEED_60, DRIVE_THRU );

        // The 2nd movement is to rotate drive train 90deg so we don't entrap the beacon cone
        // Note that while the drive train rotates 90deg one direction, the turret/lift counter-rotates
        // the OPPOSITE direction -- meaning it mostly stays pointing straight forward for this part!
        if( highJunction ) {
            robot.liftPIDPosInit(robot.LIFT_ANGLE_HIGH_A);
            robot.turretPIDPosInit(robot.TURRET_ANGLE_AUTO_R);
        } else {
            robot.liftPIDPosInit(robot.LIFT_ANGLE_MED_A);
            robot.turretPIDPosInit(robot.TURRET_ANGLE_AUTO_M_L);
        }
        autoYpos=18.0;  autoXpos=5.5;  autoAngle=-90.0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_80, TURN_SPEED_80, DRIVE_THRU );

        // Drive most of the way there very fast, and centered in the row of tiles
        autoYpos=37.0;  autoXpos=4.5;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );

        // We're close, so tilt grabber down to final scoring position
        if( highJunction ) {
            robot.grabberSetTilt(robot.GRABBER_TILT_FRONT_H_A);
        } else {
            robot.grabberSetTilt(robot.GRABBER_TILT_FRONT_M_A);
        }

        // Drive the final distance to the high junction pole at a slower/controlled speed
        if( highJunction ) {
            autoYpos=50.4;  autoXpos=8.5;
        } else {
            autoYpos=50.0;  autoXpos=2.7;
        }

        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_TO );

        // Both mechanisms should be finished, but pause here if they haven't (until they do)
        while( opModeIsActive() && ((robot.turretMotorPIDAuto == true) || (robot.liftMotorPIDAuto == true)) ) {
            performEveryLoop();
        }
*/
    } // moveToJunction

    /*--------------------------------------------------------------------------------------------*/
    private void scoreCone() {
/*
        // Start ejecting the cone
        intakeTimer.reset();
        robot.grabberSpinEject();
        // Wait for sensor to indicate it's clear (or we timeout)
        while( opModeIsActive() ) {
            performEveryLoop();
            // Ensure we eject for at least 300 msec before using sensor (in case sensor fails)
            boolean bottomSensorClear = robot.bottomConeState && (intakeTimer.milliseconds() > 300);
            // Also have a max timeout in case sensor fails
            boolean maxEjectTimeReached = (intakeTimer.milliseconds() >= 400);
            // Is cycle complete?
            if( bottomSensorClear || maxEjectTimeReached) break;
        }
        // Stop the ejector
        robot.grabberSpinStop();
        robot.grabberSetTilt( robot.GRABBER_TILT_SAFE );

        // Increment the cone we are on
        pixelNumber++;
  */
    } // scoreCone

    /*--------------------------------------------------------------------------------------------*/
    private void moveToConeStack() {
/*
        // Establish targets for turret angle (centered)
        robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );

        // Having just scored on the tall poll, turn left (-90deg) to point toward the 5-stack
        autoYpos=50.0;  autoXpos=-7.0;  autoAngle=-90.0;    // (inches, inches, degrees)
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_60, TURN_SPEED_60, DRIVE_THRU );
        robot.rotateServo.setPosition( robot.GRABBER_ROTATE_UP );

        // No longer hovering over pole so it's safe to lower to ultrasonic height (don't hook the pole!)
        robot.liftPIDPosInit( robot.LIFT_ANGLE_5STACK );

        // Drive closer to the 5-stack against the wall
        autoYpos=49.5;  autoXpos=-13.3;
        driveToPosition( autoYpos, autoXpos, autoAngle, DRIVE_SPEED_90, TURN_SPEED_80, DRIVE_TO );
        while( opModeIsActive() && ((robot.turretMotorPIDAuto == true) || (robot.liftMotorPIDAuto == true)) ) {
            performEveryLoop();
        }

 */
    } // moveToConeStack

    /*--------------------------------------------------------------------------------------------*/
    // Assumes we've already completed the alignment on the 5-stack (rotateToCenterBlueCone or
    // rotateToCenterRedCone and distanceFromFront so we're ready to actually collect the cone
    private void collectCone() {
/*
        double liftAngle5stack;
        double collectTimeout = 950.0 - (100.0 * fiveStackHeight); // cone5=450msec; cone1=850msec

        // Lower the collector to the nearly-horizontal collecting position
        robot.grabberSetTilt( robot.GRABBER_TILT_GRAB2 );

        // Determine the correct lift-angle height based on how many cones remain
        liftAngle5stack = robot.coneStackHeights[fiveStackHeight -1];

        // Lower the lift to the desired height (and ensure we're centered)
        robot.liftPIDPosInit( liftAngle5stack );
        robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );
        while( opModeIsActive() && ((robot.turretMotorPIDAuto == true) || (robot.liftMotorPIDAuto == true)) ) {
            performEveryLoop();
        }

        // Start the collector spinning
        robot.grabberSpinCollect();
        intakeTimer.reset();
        if( fiveStackHeight <= 2) {
            robot.grabberSetTilt(robot.GRABBER_TILT_GRAB);
        }
        // start to slowly lower onto cone
        robot.liftMotorsSetPower( -0.25 );
        while(robot.topConeState && intakeTimer.milliseconds() <= collectTimeout) {
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

 */
    } // collectCone

    /*--------------------------------------------------------------------------------------------*/
    private void moveToJunctionFromStack( boolean highJunction, boolean keepCone) {
/*
        // Perform setup to center turret and raise lift to scoring position
        if( !keepCone ) {
            if (highJunction) {
                autoYpos = 48.4;
                autoXpos = 8.5;
                autoAngle = -90.0;    // (inches, inches, degrees)
                robot.turretPIDPosInit(robot.TURRET_ANGLE_5STACK_L);
                robot.liftPIDPosInit(robot.LIFT_ANGLE_HIGH_A);
                robot.grabberSetTilt(robot.GRABBER_TILT_FRONT_H_A);
            } else {
                autoYpos = 50.0;
                autoXpos = 2.7;
                robot.turretPIDPosInit(robot.TURRET_ANGLE_AUTO_M_L);
                robot.liftPIDPosInit(robot.LIFT_ANGLE_MED_A);
                robot.grabberSetTilt(robot.GRABBER_TILT_FRONT_M_A);
            }
        }

        // Drive back to tall junction (adjusting lift along the way)
        // (stay along Y=51.5 instead of returning to Y=54.0, but rotate turret more (-56.5, not -34.5)
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
*/
    } // moveToTallJunctionFromStack

    /*--------------------------------------------------------------------------------------------*/
    /* +---+---H---+     H = Tall/High junction pole on RIGHT                                     */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* | 1 | 2 | 3 |                                                                              */
    /* +---+---+---+                                                                              */
    /* |   | S |   |     S = Starting floor tile                                                  */
    /* +---+---+---/                                                                              */
    private void signalZoneParking( int signalZoneLocation, boolean parkWithCone ) {
/*
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

        // Initialize so that lift lowers to driving position (or low-scoring positipn)
        robot.liftPIDPosInit( (parkWithCone)? robot.LIFT_ANGLE_LOW : robot.LIFT_ANGLE_COLLECT );

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
*/
    } // signalZoneParking

} /* AutonomousLeftBlue */
