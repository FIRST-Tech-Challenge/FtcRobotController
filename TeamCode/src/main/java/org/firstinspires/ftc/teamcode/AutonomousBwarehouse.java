/* FTC Team 7572 - Version 1.2 (03/02/2022)
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
@Autonomous(name="Autonomous Blue (warehouse)", group="7592", preselectTeleOp = "Teleop-Blue")
//@Disabled
public class AutonomousBwarehouse extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y              = true;    // Drive forward/backward
    static final boolean DRIVE_X              = false;   // Drive right/left (not DRIVE_Y)

    static final double  DRIVE_SPEED_10       = 0.10;    // Lower speed for moving from a standstill
    static final double  DRIVE_SPEED_20       = 0.20;    // Lower speed for moving from a standstill
    static final double  DRIVE_SPEED_30       = 0.30;    // Lower speed for fine control going sideways
    static final double  DRIVE_SPEED_40       = 0.40;    // Normally go slower to achieve better accuracy
    static final double  DRIVE_SPEED_55       = 0.55;    // Somewhat longer distances, go a little faster
    static final double  TURN_SPEED_20        = 0.20;    // Nominal half speed for better accuracy.

    static final double  HEADING_THRESHOLD    = 2.0;     // Minimum of 1 degree for an integer gyro
    static final double  P_TURN_COEFF         = 0.050;   // Larger is more responsive, but also less stable

    static final int     DRIVE_THRU           = 2;       // COAST after the specified movement

    double    sonarRangeL=0.0, sonarRangeR=0.0, sonarRangeF=0.0, sonarRangeB=0.0;

    OpenCvCamera webcam;
    public int hubLevel = 0;   // dynamic (gets updated every cycle during INIT)
    public static double collisionDelay = 4.0;  // wait 4 seconds before moving (to avoid collision)
    private ElapsedTime autoTimer = new ElapsedTime();

    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;

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
                webcam.setPipeline(new FreightFrenzyPipeline(false, false));
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        int redAlignedCount;
        int blueAlignedCount;
		
        // Wait for the game to start (driver presses PLAY).  While waiting, poll for team color/number
        while (!isStarted()) {
            sonarRangeR = robot.updateSonarRangeR();
            checkForNewCollisionDelay();
            telemetry.addData("ALLIANCE", "%s", "BLUE (warehouse)");
            telemetry.addData("Hub Level", "%d", hubLevel);
            telemetry.addData("Sonar Range", "%.1f inches (52.8?)", sonarRangeR/2.54 );
            telemetry.addData("Start delay", "%.1f seconds (dpad up/down)", collisionDelay );
            telemetry.addData("Left Blue Alignment", "%d %b", FreightFrenzyPipeline.leftBlueAverage, FreightFrenzyPipeline.alignedBlueLeft);
            telemetry.addData("Center Blue Alignment", "%d %b", FreightFrenzyPipeline.centerBlueAverage, FreightFrenzyPipeline.alignedBlueCenter);
            telemetry.addData("Right Blue Alignment", "%d %b", FreightFrenzyPipeline.rightBlueAverage, FreightFrenzyPipeline.alignedBlueRight);
            redAlignedCount = (FreightFrenzyPipeline.alignedRedLeft ? 1 : 0);
            redAlignedCount += (FreightFrenzyPipeline.alignedRedCenter ? 1 : 0);
            redAlignedCount += (FreightFrenzyPipeline.alignedRedRight ? 1 : 0);
            blueAlignedCount = (FreightFrenzyPipeline.alignedBlueLeft ? 1 : 0);
            blueAlignedCount += (FreightFrenzyPipeline.alignedBlueCenter ? 1 : 0);
            blueAlignedCount += (FreightFrenzyPipeline.alignedBlueRight ? 1 : 0);
            if(blueAlignedCount >= 2) {
                telemetry.addLine("Blue aligned for blue autonomous. Good job!");
                hubLevel = FreightFrenzyPipeline.hubLevel;
            } else if(redAlignedCount >= 2) {
                telemetry.addLine("****************************************************");
                telemetry.addLine("* WARNING: Red aligned for BLUE autonomous. *");
                telemetry.addLine("*          Something is wrong, so so wrong!             *");
                telemetry.addLine("****************************************************");
            } else {
                telemetry.addLine("Robot is not aligned for autonomous. Robot so confused!");
            }
            telemetry.update();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Start our autonomous timer.
        autoTimer.reset();

        // Sampling is completed during the INIT stage; No longer need camera active/streaming
        webcam.stopStreaming();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
            hubLevel = FreightFrenzyPipeline.hubLevel;
            FreightFrenzyPipeline.saveLastAutoImage();
        }

        webcam.closeCameraDevice();

        //---------------------------------------------------------------------------------
        // UNIT TEST: The following methods verify our basic robot actions.
        // Comment them out when not being tested.
//      testGyroDrive();
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
    private void checkForNewCollisionDelay() {
        gamepad1_dpad_up_last   = gamepad1_dpad_up_now;
        gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last = gamepad1_dpad_down_now;
        gamepad1_dpad_down_now  = gamepad1.dpad_down;
        if( gamepad1_dpad_up_now && !gamepad1_dpad_up_last) {
            if( collisionDelay < 5.0 )
                collisionDelay += 1.0;
        } // increment
        if( gamepad1_dpad_down_now && !gamepad1_dpad_down_last) {
            if( collisionDelay > 0.0 )
                collisionDelay -= 1.0;
        } // decrement
    } // checkForNewCollisionDelay

    /*--------------------------------------------------------------------------------------------*/
    private void testGyroDrive() {
        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, 24.0, 999.9, DRIVE_THRU ); // Drive FWD 24" along current heading
        gyroDrive(DRIVE_SPEED_30, DRIVE_X, 24.0, 999.9, DRIVE_THRU ); // Strafe RIGHT 24" along current heading
        gyroTurn(TURN_SPEED_20, (getAngle() + 90.0) );       // Turn CW 90 Degrees
    } // testGyroDrive

    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

        boolean freightCollected = false;
        double freightCollectAngle = 75.0;

        // Wait before moving to avoid collision with duck-side alliance partner
        // trying to dump a freight in the alliance shipping hub
        sleep( (long)( collisionDelay * 1000 ) );

        // Drive forward and collect the team element off the floor
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "collectTeamElement");
            telemetry.update();
            collectTeamElement( hubLevel );
        }

        // Drive to the alliance hub to deposit freight
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "moveToHub");
            telemetry.update();
            moveToHub( hubLevel );
        }

        // Deposit freight in top/middle/bottom
        if( opModeIsActive() ) {
            telemetry.addData("Skill", "dumpFreight");
            telemetry.update();
            dumpFreight( hubLevel );
        }

        // Drive into warehouse
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "driveToWarehouse");
            telemetry.update();
            driveToWarehouse(hubLevel);
        }

        // Perform first freight collection
        while( opModeIsActive() && !freightCollected) {
            // Drive into freight pile to collect
            telemetry.addData("Skill", "collectFreight " + freightCollectAngle);
            telemetry.update();
            freightCollected = collectFreight(hubLevel, freightCollectAngle, 350 );
            freightCollectAngle -= 5.0;  // try again at a slightly different angle
        }

        // Score the freight if we have collected one, and we have enough time.
//      if(opModeIsActive() && freightCollected && (autoTimer.milliseconds() <= HUB_SCORE_TIME_THRESHOLD)) {
//         telemetry.addData("Skill", "scoreFreightAllianceHub");
//         telemetry.update();
//         freightCollected = !scoreFreightAllianceHub(hubLevel);
//         freightCollectAngle = 35.0;
//      }

        // Collect second freight if we're not still holding the first one
//      while( opModeIsActive() && !freightCollected) {
//          // Drive into freight pile to collect
//          telemetry.addData("Skill", "collectFreight " + freightCollectAngle);
//          telemetry.update();
//          freightCollected = collectFreight(hubLevel, freightCollectAngle, 250 );
//          freightCollectAngle -= 5.0;  // try again at a slightly different angle
//      }
    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void collectTeamElement( int level ) {
        double strafeDist1 = 0.0;
        double turnAngle = 0.0;
        double distanceToGrab = 3.2;
        double strafeDist2 = 0.0;

        switch( level ) {
            case 3 : strafeDist1 = 0.0;      // right/top
                     turnAngle = 0.0;
                     distanceToGrab = -2.5;
                     strafeDist2 = -3.0;
                     break;
            case 2 : strafeDist1 = -8.0;     // middle/middle
                     turnAngle = -42.0;
                     distanceToGrab = -3.0;
                     strafeDist2 = 0.0;
                     break;
            case 1 : strafeDist1 = -8.0;
                     turnAngle = -52.0;
                     distanceToGrab = -7.0; // left/bottom
                     strafeDist2 = 0.0;
                     break;
        } // switch()

        // Move forward away from field wall so it's safe to raise the arms
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -4.2, 0.0, DRIVE_TO );

        // Command capping arm into the grabbing position
        robot.cappingArmPosInit( robot.CAPPING_ARM_POS_GRAB );
        robot.freightArmPosInit( robot.FREIGHT_ARM_POS_SPIN );

        // Process the first 750 msec of motion
        ElapsedTime fieldWallTimer = new ElapsedTime();
        fieldWallTimer.reset();  // start now
        while( opModeIsActive() && (fieldWallTimer.milliseconds() < 750) ) {
            performEveryLoop();
        }

        // We're now a safe distance from the wall to rotate the wrist and open the claw
        robot.clawServo.setPosition( robot.CLAW_SERVO_OPEN );    // open claw
        robot.wristPositionAuto( robot.WRIST_SERVO_GRAB );       // rotate wrist into the grab position
        robot.boxServo.setPosition( robot.BOX_SERVO_TRANSPORT );

        // Finish both arm movements before continuing
        while( opModeIsActive() &&
                ((robot.cappingMotorAuto == true) || (robot.freightMotorAuto == true)) ) {
            performEveryLoop();
        }

        // Strafe sideways (can't ROTATE because rear wheels will hit the barrier)
        if( Math.abs(strafeDist1) > 0.10 ) {
            gyroDrive(DRIVE_SPEED_30, DRIVE_X, strafeDist1, 0.0, DRIVE_THRU );
            robot.stopMotion();
        }
        if( Math.abs(turnAngle) > 0.10 ) {
            gyroTurn(TURN_SPEED_20, turnAngle );
        }

        // Drive forward to collect the element
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, distanceToGrab, 999.9, DRIVE_TO );
        robot.clawServo.setPosition( robot.CLAW_SERVO_CLOSED );    // close claw
        sleep( 500 );   // wait for claw to close

        // With the team element in hand, raise both arms (opposite directions)
        robot.cappingArmPosInit( robot.CAPPING_ARM_POS_LIBERTY );
        robot.wristPositionAuto( robot.WRIST_SERVO_LIBERTY );  // store position (handles unpowered!)
        robot.freightArmPosInit( robot.FREIGHT_ARM_POS_VERTICAL );

        // Strafe sideways (can't ROTATE because rear wheels will hit the barrier)
        if( Math.abs(strafeDist2) > 0.10 ) {
            gyroDrive(DRIVE_SPEED_30, DRIVE_X, strafeDist2, 0.0, DRIVE_THRU );
            robot.stopMotion();
        }

        // Finish both arm movements before continuing
        while( opModeIsActive() &&
                ((robot.cappingMotorAuto == true) || (robot.freightMotorAuto == true)) ) {
            performEveryLoop();
        }

    } // collectTeamElement

    /*--------------------------------------------------------------------------------------------*/
    private void moveToHub( int level ) {
        double angleToHub = 0.0;
        double distanceToHub = 0.0;
        int    timeToHub = 1000;
        int    freightArmPos = 0;

        switch( level ) {
            case 3 : angleToHub = 35.0;    // top
                     distanceToHub = -6.8;
                     timeToHub = 0;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_TOP_AUTO;
                     break;
            case 2 : angleToHub = 30.0;
                     distanceToHub = -3.0;  // middle
                     timeToHub = 300;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_MIDDLE_AUTO;
                     break;
            case 1 : angleToHub = 32.0;
                     distanceToHub = 0.0;  // bottom
                     timeToHub = 550;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_BOTTOM_AUTO;
                     break;
        } // switch()

        // Start arm motion
        robot.freightArmPosInit( freightArmPos );

        // Turn toward hub
        double currentAngle = robot.headingIMU();
        if( Math.abs(angleToHub-currentAngle) > 2.0 )
            gyroTurn(TURN_SPEED_20, angleToHub );

        // Drive partially forward
        if( Math.abs(distanceToHub) > 0.0 ) {
            gyroDrive(DRIVE_SPEED_30, DRIVE_Y, distanceToHub, angleToHub, DRIVE_TO);
        }

        // Ensure arm has reached it's final position
        while( opModeIsActive() && (robot.freightMotorAuto == true) ) {
            performEveryLoop();
        }

        // Drive forward the final amount
        if(timeToHub > 0) {
            timeDriveStraight(-DRIVE_SPEED_30, timeToHub);
        }

  } // moveToHub

    /*--------------------------------------------------------------------------------------------*/
    private void dumpFreight(int level ) {
        double servoPos = robot.BOX_SERVO_DUMP_TOP;
        int moveAwayTime = 200;

        switch( level ) {
            case 3 : servoPos = robot.BOX_SERVO_DUMP_TOP;
                     moveAwayTime = 0;
                     break;
            case 2 : servoPos = robot.BOX_SERVO_DUMP_MIDDLE;
                     moveAwayTime = 0;
                     break;
            case 1 : servoPos = robot.BOX_SERVO_DUMP_BOTTOM;
                     moveAwayTime = 200;
                     break;
        } // switch()

        robot.boxServo.setPosition( servoPos );     // rotate the box to dump
        sleep( 500 );                               // let cube drop out

        // back away and store arm
        if(moveAwayTime > 0) {
            timeDriveStraight(DRIVE_SPEED_30, moveAwayTime);
        }

        gyroTurn(TURN_SPEED_20, 90.0 );   // Turn toward the freight warehouse
        robot.boxServo.setPosition( robot.BOX_SERVO_COLLECT );
        robot.freightArmPosInit( robot.FREIGHT_ARM_POS_TRANSPORT1 );
        robot.cappingArmPosInit( robot.CAPPING_ARM_POS_STORE );
        robot.wristPositionAuto( robot.WRIST_SERVO_STORE );  // store position (handles unpowered!)

    } // dumpFreight

    /*--------------------------------------------------------------------------------------------*/
    private void driveToWarehouse( int level  ) {
        double warehouseDistance = 0.0;
        switch( level ) {
            case 3 : warehouseDistance = 39.0;  break; // Top
            case 2 : warehouseDistance = 41.5;  break; // Middle
            case 1 : warehouseDistance = 45.0;  break; // Bottom
        } // switch()

        gyroDrive(DRIVE_SPEED_40, DRIVE_Y, warehouseDistance, 999.9, DRIVE_THRU );
        robot.stopMotion();
        // Ensure the arm motion has completed before continuing
        while( opModeIsActive() && (robot.freightMotorAuto == true) ) {
            performEveryLoop();
        }

    } // driveToWarehouse

    /*--------------------------------------------------------------------------------------------*/
    private boolean collectFreight(int level, double angle, int backupTime) {
        boolean collected = false;
        double slowlyCollectMyPrecious = 0.12;
        int freightDetections = 0;
        ElapsedTime freightTimeout = new ElapsedTime();

        robot.boxServo.setPosition( robot.BOX_SERVO_COLLECT );
        robot.freightArmPosInit( robot.FREIGHT_ARM_POS_COLLECT );
        gyroTurn(TURN_SPEED_20, angle );   // Turn toward the freight
        // If the arm didn't reach COLLECT during the turn, wait a little longer
        while( opModeIsActive() && (robot.freightMotorAuto == true) ) {
            performEveryLoop();
        }
        // With freight-arm down, it's safe to lower the intake
        robot.linkServo.setPosition(robot.LINK_SERVO_LOWERED);
        robot.sweepMotor.setPower(1.0);
        robot.driveTrainMotors( slowlyCollectMyPrecious, (slowlyCollectMyPrecious + 0.02),
                                slowlyCollectMyPrecious, (slowlyCollectMyPrecious + 0.02) );

        freightTimeout.reset();
        while((opModeIsActive()) && (freightTimeout.milliseconds() < 2750) && (freightDetections < HardwareBothHubs.FREIGHT_DETECTED_THRESHOLD)) {
            if(robot.freightPresent()) {
                freightDetections++;
            } else {
                freightDetections = 0;
            }
        }
        robot.stopMotion();
        robot.sweepMotor.setPower(0.0);
        if(freightDetections >= 1) {
            collected = true;
            // Raise the sweeper so that we are free to lift the arm
            robot.linkServo.setPosition( robot.LINK_SERVO_RAISED );
            // Partially rotate the box servo toward the  transport position
            // (allows any 2nd piece of freight to fall off before we rotate
            //  so far that the keep-it-in-erator bar could trap it.
            robot.boxServo.setPosition( robot.BOX_SERVO_STORED );
            // Wait for collector arm to move out of the way
            sleep(300);
            // Lift the freight-arm so we're clear to drive over the barrier
            robot.freightArmPosInit( robot.FREIGHT_ARM_POS_TRANSPORT1 );
            // Process the first 300 msec of freight-arm lifting motion
            ElapsedTime armLiftTimer = new ElapsedTime();
            armLiftTimer.reset();  // start now
            while( opModeIsActive() && (armLiftTimer.milliseconds() < 300) ) {
                performEveryLoop();
            }
            // Now fully rotate the box servo into transport position
            robot.boxServo.setPosition(robot.BOX_SERVO_TRANSPORT);
        }

        // Drive backward away from the pile
        timeDriveStraight(-DRIVE_SPEED_30, backupTime);

        // Wait for any arm-lifting motion to complete
        while( opModeIsActive() && (robot.freightMotorAuto == true) ) {
            performEveryLoop();
        }

        return collected;
    } // collectFreight

    /*--------------------------------------------------------------------------------------------*/
    boolean scoreFreightAllianceHub(int level) {
        boolean scored = false;
        final double RAMMING_SPEED = DRIVE_SPEED_30;

        // Not sure what value this should be. This is the minimum distance we want.
        final double MIN_WALL_DISTANCE = 10.0;  // 16", but we're at an angle and there's some error

        gyroTurn(TURN_SPEED_20, -175.0 );  // Turn toward shared hub (angled away from triangle)

        double wallDistance1 = leftRangeSensor()/2.54;
        telemetry.addData("wallDistance1", "%.1f  in", wallDistance1 );
        telemetry.update();

        if(wallDistance1 < MIN_WALL_DISTANCE) {
            // This is the optimal distance we want. If we are moving, lets make it count.
            double distanceToGo = (MIN_WALL_DISTANCE + 2.0) - wallDistance1;
            distanceToGo = Math.max( distanceToGo, 10.0 ); // don't go more than 10"
            // Need to verify which way to strafe.
            gyroDrive(DRIVE_SPEED_40, DRIVE_X, distanceToGo, 999.9, DRIVE_THRU);
            robot.stopMotion();
        }

        // Make sure there isn't something wrong where the robot can't move properly.
        double wallDistance2 = leftRangeSensor()/2.54;
        telemetry.addData("wallDistance1", "%.1f  in", wallDistance1 );
        telemetry.addData("wallDistance2", "%.1f  in", wallDistance2 );
        telemetry.update();

        if(wallDistance2 > MIN_WALL_DISTANCE) {
            // Update our tilt angle information
            robot.driveTrainMotors(RAMMING_SPEED, RAMMING_SPEED, RAMMING_SPEED, RAMMING_SPEED);
            robot.headingIMU();
            while (opModeIsActive() && (robot.tiltAngle >= HardwareBothHubs.BARRIER_NESTED_ROBOT_TILT_AUTO)) {
                robot.headingIMU();
            }
            // stop and wait for it to settle
            robot.stopMotion();
            sleep(250);

            // We should be nestled, strafe over and dump freight
            if (opModeIsActive()) {
                timeDriveStrafe(DRIVE_SPEED_30, 1000);

                robot.boxServo.setPosition(robot.BOX_SERVO_DUMP_FRONT);
                sleep(500);
                scored = true;

                // Backup into warehouse
                gyroDrive(DRIVE_SPEED_40, DRIVE_Y, -23.0, 999.9, DRIVE_THRU);
                robot.stopMotion();
            }
        } // walldistance2

        return scored;
    } // scoreFreightAllianceHub

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Capture range-sensor data (one reading! call from main control loop)  */
    /*                                                                                 */
    /*  Designed for test programs that are used to assess the mounting location of    */
    /*  your sensors and whether you get reliable/repeatable returns off various field */
    /*  elements.                                                                      */
    /*                                                                                 */
    /*  IMPORTANT!! updateSonarRangeL / updateSonarRangeR may call getDistanceSync(),  */
    /*  which sends out an ultrasonic pulse and SLEEPS for the sonar propogation delay */
    /*  (50 sec) before reading the range result.  Don't use in applications where an  */
    /*  extra 50/100 msec (ie, 1 or 2 sensors) in the loop time will create problems.  */
    /*  If getDistanceAsync() is used, then this warning doesn't apply.                */
    /*---------------------------------------------------------------------------------*/
    void processRangeSensors() {
        sonarRangeL = robot.updateSonarRangeL();
        sonarRangeR = robot.updateSonarRangeR();
        sonarRangeF = robot.updateSonarRangeF();
        sonarRangeB = robot.updateSonarRangeB();
    } // processRangeSensors

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: averaged range-sensor data (multiple readings!)                       */
    /*                                                                                 */
    /*  Designed for applications where continuous range updates are unnecessary, but  */
    /*  we want to know the correct distance "right now".                              */
    /*---------------------------------------------------------------------------------*/
    void averagedRangeSensors() {
        // repeatedly update all 4 readings.  Each loop adds a reading to the
        // internal array from which we return the new MEDIAN value.
        for( int i=0; i<5; i++ ) {
            sonarRangeL = robot.updateSonarRangeL();
            sonarRangeR = robot.updateSonarRangeR();
            sonarRangeF = robot.updateSonarRangeF();
            sonarRangeB = robot.updateSonarRangeB();
        }
    } // averagedRangeSensors

    /*---------------------------------------------------------------------------------*/
    double leftRangeSensor() {
        for( int i=0; i<5; i++ ) {
            sonarRangeL = robot.updateSonarRangeL();
            sleep(50);
        }
        return sonarRangeL;
    } // leftRangeSensor

    /*---------------------------------------------------------------------------------*/
    double rightRangeSensor() {
        for( int i=0; i<5; i++ ) {
            sonarRangeR = robot.updateSonarRangeR();
            sleep(50);
        }
        return sonarRangeR;
    } // rightRangeSensor

    /*---------------------------------------------------------------------------------*/
    double backRangeSensor() {
        for( int i=0; i<6; i++ ) {
            sonarRangeB = robot.updateSonarRangeB();
            sleep(50);
        }
        return sonarRangeB;
    } // backRangeSensor

    /*---------------------------------------------------------------------------------*/
    double computeDriveAngle( double x0, double x1, double y0, double y1 ) {
        double deltaX = (x1 - x0);
        double deltaY = (y1 - y0);  // must drive at least 10 cm (also avoids trig error)
        double driveAngle = (deltaY < 10.0)? 0.0 : Math.atan2(deltaX,deltaY);  // radians
        driveAngle = driveAngle * (180.0 / Math.PI);  // degrees
        return driveAngle;
    } // computeDriveAngle

} /* AutonomousBwarehouse */
