/* FTC Team 7572 - Version 1.1 (12/16/2021)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
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
@Autonomous(name="Autonomous Red (warehouse)", group="7592", preselectTeleOp = "Teleop-Red")
//@Disabled
public class AutonomousRwarehouse extends AutonomousBase {

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
    public int blockLevel = 0;   // dynamic (gets updated every cycle during INIT)
    public static double collisionDelay = 4.0;  // wait 4 seconds before moving (to avoid collision)

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
                webcam.setPipeline(new FreightFrenzyPipeline(true, false));
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
            sonarRangeL = robot.updateSonarRangeL();
            checkForNewCollisionDelay();
            telemetry.addData("ALLIANCE", "%s", "RED (warehouse)");
            telemetry.addData("Block Level", "%d", blockLevel );
            telemetry.addData("Sonar Range", "%.1f inches (50.4?)", sonarRangeL/2.54 );
            telemetry.addData("Start delay", "%.1f seconds (dpad up/down)", collisionDelay );
            telemetry.addData("Left Red Alignment", "%d %b", FreightFrenzyPipeline.leftRedAverage, FreightFrenzyPipeline.alignedRedLeft);
            telemetry.addData("Center Red Alignment", "%d %b", FreightFrenzyPipeline.centerRedAverage, FreightFrenzyPipeline.alignedRedCenter);
            telemetry.addData("Right Red Alignment", "%d %b", FreightFrenzyPipeline.rightRedAverage, FreightFrenzyPipeline.alignedRedRight);
            redAlignedCount = (FreightFrenzyPipeline.alignedRedLeft ? 1 : 0);
            redAlignedCount += (FreightFrenzyPipeline.alignedRedCenter ? 1 : 0);
            redAlignedCount += (FreightFrenzyPipeline.alignedRedRight ? 1 : 0);
            blueAlignedCount = (FreightFrenzyPipeline.alignedBlueLeft ? 1 : 0);
            blueAlignedCount += (FreightFrenzyPipeline.alignedBlueCenter ? 1 : 0);
            blueAlignedCount += (FreightFrenzyPipeline.alignedBlueRight ? 1 : 0);
            if(redAlignedCount >= 2) {
                telemetry.addLine("Red aligned for red autonomous. Good job!");
                blockLevel = FreightFrenzyPipeline.blockLevel;
            } else if (blueAlignedCount >= 2) {
                telemetry.addLine("****************************************************");
                telemetry.addLine("* WARNING: Blue aligned for RED autonomous. *");
                telemetry.addLine("*          Something is wrong, so so wrong!             *");
                telemetry.addLine("****************************************************");
            } else {
                telemetry.addLine("Robot is not aligned for autonomous. Robot so confused!");
            }
            telemetry.update();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Sampling is completed during the INIT stage; No longer need camera active/streaming
        webcam.stopStreaming();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
            blockLevel = FreightFrenzyPipeline.blockLevel;
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
        // Wait before moving to avoid collision with duck-side alliance partner
        // trying to dump a block in the alliance shipping hub
        sleep( (long)( collisionDelay * 1000 ) );

        // Drive forward and collect the team element off the floor
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "collectTeamElement");
            telemetry.update();
            collectTeamElement( blockLevel );
        }

        // Drive to the alliance hub to deposit block
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "moveToHub");
            telemetry.update();
            moveToHub( blockLevel );
        }

        // Deposit block in top/middle/bottom
        if( opModeIsActive() ) {
            telemetry.addData("Skill", "dumpBlock");
            telemetry.update();
            dumpBlock( blockLevel );
        }

        // Drive to warehouse to park
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "driveToWarehouse");
            telemetry.update();
            driveToWarehouse( blockLevel );
        }

        // Drive into freight pile to collect
        if( opModeIsActive() ) {
            telemetry.addData("Skill", "collectFreight -45");
            telemetry.update();
            freightCollected = collectFreight( blockLevel, -45.0 );
        }

        // If we collected a freight, dump it in shared hub
        if( opModeIsActive() && freightCollected) {
            telemetry.addData("Skill", "scoreFreightSharedHub");
            telemetry.update();
            scoreFreightSharedHub( blockLevel );
        }

        // Drive into freight pile to collect a second freight
        if( opModeIsActive() ) {
            telemetry.addData("Skill", "collectFreight1 -55");
            telemetry.update();
            collectFreight( blockLevel, -55.0 );
        }
    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void collectTeamElement( int level ) {
        double strafeDist = 0.0;
        double distanceToGrab = 3.2;

        switch( level ) {
            case 3 : strafeDist = 0.0;      // right/top
                     distanceToGrab = -2.0;
                     break;
            case 2 : strafeDist = 7.0;     // middle/middle
                     distanceToGrab = -2.0;
                     break;
            case 1 : strafeDist = 15.0;
                     distanceToGrab = -2.5; // left/bottom
                     break;
        } // switch()

        // Move forward away from field wall so it's safe to raise the arms
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -4.2, 0.0, DRIVE_TO );

        // Rotate the capping arm into the grabbing position
        robot.cappingArmPosition( robot.CAPPING_ARM_POS_GRAB, 0.50 );
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_SPIN, 0.50 );
        sleep( 750);   // wait for capping arm to clear the field wall
        robot.clawServo.setPosition( robot.CLAW_SERVO_OPEN );    // open claw
        robot.wristPositionAuto( robot.WRIST_SERVO_GRAB );       // rotate wrist into the grab position
        robot.boxServo.setPosition( robot.BOX_SERVO_TRANSPORT );
        sleep( 2000);   // wait for arm to reach final position

        // Strafe sideways (can't ROTATE because rear wheels will hit the barrier)
        if( Math.abs(strafeDist) > 0.10 ) {
            gyroDrive(DRIVE_SPEED_30, DRIVE_X, strafeDist, 0.0, DRIVE_TO );
        }

        // Drive forward to collect the element
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, distanceToGrab, 999.9, DRIVE_TO );
        robot.clawServo.setPosition( robot.CLAW_SERVO_CLOSED );    // close claw
        sleep( 500 );   // wait for claw to close

        robot.cappingArmPosition( robot.CAPPING_ARM_POS_LIBERTY, 0.40 );
        robot.wristPositionAuto( robot.WRIST_SERVO_LIBERTY );  // store position (handles unpowered!)
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_VERTICAL, 0.40 );
    } // collectTeamElement

    /*--------------------------------------------------------------------------------------------*/
    private void moveToHub( int level ) {
        double angleToHub = 0.0;
        double distanceToHub = 0.0;
        double finalDistanceToHub = 0.0;

        int    freightArmPos = 0;
        long   armSleep = 0;

        switch( level ) {
            case 3 : angleToHub = -38.0;    // top
                     distanceToHub = -6.2;
                     finalDistanceToHub = 0.0;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_TOP_AUTO;
                     armSleep = 0;
                     break;
            case 2 : angleToHub = -38.0;
                     distanceToHub = -3.0;  // middle
                     finalDistanceToHub = -3.5;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_MIDDLE_AUTO;
                     armSleep = 750;  // 750 msec
                     break;
            case 1 : angleToHub = -27.0;
                     distanceToHub = -1.0;  // bottom
                     finalDistanceToHub = -2.5;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_BOTTOM_AUTO;
                     armSleep = 2000;   // 2.0 sec
                     break;
        } // switch()

        double currentAngle = robot.headingIMU();

        robot.freightArmPosition( freightArmPos, 0.50 );

        if( Math.abs(angleToHub-currentAngle) > 2.0 )
            gyroTurn(TURN_SPEED_20, angleToHub );

        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, distanceToHub, angleToHub, DRIVE_TO );

        if( armSleep > 0 ) {
            sleep( armSleep );
        }
        if( Math.abs(finalDistanceToHub) > 0 ) {
            gyroDrive(DRIVE_SPEED_30, DRIVE_Y, finalDistanceToHub, angleToHub, DRIVE_TO );
        }

  } // moveToHub

    /*--------------------------------------------------------------------------------------------*/
    private void dumpBlock( int level ) {
        double servoPos = robot.BOX_SERVO_DUMP_TOP;
        double backDistance = 3.0;

        switch( level ) {
            case 3 : servoPos = robot.BOX_SERVO_DUMP_TOP;
                     backDistance = 4.5;
                     break;
            case 2 : servoPos = robot.BOX_SERVO_DUMP_MIDDLE;
                     backDistance = 3.5;
                     break;
            case 1 : servoPos = robot.BOX_SERVO_DUMP_BOTTOM;
                     backDistance = 3.5;
                     break;
        } // switch()

        robot.boxServo.setPosition( servoPos );     // rotate the box to dump
        sleep( 1500 );                    // let cube drop out
        // back away and store arm
        if( level == 1 ){
            gyroDrive(DRIVE_SPEED_30, DRIVE_Y, 2.0, 999.9, DRIVE_TO );
        }
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_TRANSPORT1, 0.50 );
        robot.boxServo.setPosition( robot.BOX_SERVO_COLLECT );

        robot.cappingArmPosition( robot.CAPPING_ARM_POS_STORE, 0.40 );
        robot.wristPositionAuto( robot.WRIST_SERVO_STORE );  // store position (handles unpowered!)
        if(level == 1) {
            gyroTurn(DRIVE_SPEED_30, -135.0);
            gyroDrive(DRIVE_SPEED_30, DRIVE_Y, 6.0, 999.9, DRIVE_TO );
        }
    } // dumpBlock

    /*--------------------------------------------------------------------------------------------*/
    private void driveToWarehouse( int level  ) {
        double warehouseDistance = 0.0;
        switch( level ) {
            case 3 : warehouseDistance = 50.0;  break;
            case 2 : warehouseDistance = 54.0;  break;
            case 1 : warehouseDistance = 52.0;  break;
        } // switch()

        gyroTurn(TURN_SPEED_20, -90.0 );   // Turn toward the freight warehouse
        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, warehouseDistance, 999.9, DRIVE_TO );
    } // driveToWarehouse


    /*--------------------------------------------------------------------------------------------*/
    private boolean collectFreight(int level, double angle) {
        boolean freightCollected = false;
        double slowlyCollectMyPrecious = 0.12;
        int freightDetections = 0;
        ElapsedTime freightTimeout = new ElapsedTime();

        robot.boxServo.setPosition( robot.BOX_SERVO_COLLECT );
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_COLLECT, 0.50 );
        gyroTurn(TURN_SPEED_20, angle );   // Turn toward the freight
        // Turning time should be plenty of time for arm to reach collect position so we
        // can lower the intake.
        robot.linkServo.setPosition(robot.LINK_SERVO_LOWERED);
        robot.sweepMotor.setPower(1.0);
        robot.driveTrainMotors( slowlyCollectMyPrecious, slowlyCollectMyPrecious,
                                slowlyCollectMyPrecious, slowlyCollectMyPrecious );

        freightTimeout.reset();
        while((opModeIsActive()) && (freightTimeout.milliseconds() < 2500) && (freightDetections < HardwareBothHubs.FREIGHT_COLLECTED)) {
            if(robot.freightPresent()) {
                freightDetections++;
            } else {
                freightDetections = 0;
            }
        }
        robot.stopMotion();
        robot.sweepMotor.setPower(0.0);
        if(freightDetections >= 1) {
            freightCollected = true;
            robot.linkServo.setPosition(robot.LINK_SERVO_RAISED);
            robot.boxServo.setPosition(robot.BOX_SERVO_TRANSPORT);
            sleep(300);
            robot.freightArmPosition(robot.FREIGHT_ARM_POS_TRANSPORT1, 0.50);
            sleep(750);
        }
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -5.0, 999.9, DRIVE_TO );

        return freightCollected;
    } // collectFreight

    void scoreFreightSharedHub(int level) {
        gyroTurn(TURN_SPEED_20, -180.0 );   // Turn toward the shared hub
        // Update our tilt angle information
        robot.driveTrainMotors( 0.4, 0.4, 0.4, 0.4);
        robot.headingIMU();
        while(opModeIsActive() && (robot.tiltAngle > HardwareBothHubs.SHARED_HUB_TILT)) {
            robot.headingIMU();
        }
        // We are nestled, dump freight
        if(opModeIsActive()) {
            robot.boxServo.setPosition(robot.BOX_SERVO_DUMP_FRONT);
            sleep(500);

            // Backup into warehouse
            gyroDrive(DRIVE_SPEED_30, DRIVE_Y, -12.0, 999.9, DRIVE_TO);
        }
    } // scoreFreightSharedHub

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

} /* AutonomousRwarehouse */
