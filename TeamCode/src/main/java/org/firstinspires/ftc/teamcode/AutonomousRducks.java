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
@Autonomous(name="Autonomous Red (ducks)", group="7592", preselectTeleOp = "Teleop-Red")
@Disabled
public class AutonomousRducks extends AutonomousBase {

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

    static final int     DRIVE_THRU           = 2;       // COAST after the specified movement

    double    sonarRangeL=0.0, sonarRangeR=0.0, sonarRangeF=0.0, sonarRangeB=0.0;

    OpenCvCamera webcam;
    public int blockLevel = 0;   // dynamic (gets updated every cycle during INIT)

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
                webcam.setPipeline(new FreightFrenzyPipeline(true, true));
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
            telemetry.addData("ALLIANCE", "%s", "RED (ducks)");
            telemetry.addData("Block Level", "%d", blockLevel );
            telemetry.addData("Sonar Range", "%.1f inches (28.0)", sonarRangeR/2.54 );
            telemetry.addData("Left Red Alignment", "%.2f %b", FreightFrenzyPipeline.leftRedAverage, FreightFrenzyPipeline.alignedRedLeft);
            telemetry.addData("Center Red Alignment", "%.2f %b", FreightFrenzyPipeline.centerRedAverage, FreightFrenzyPipeline.alignedRedCenter);
            telemetry.addData("Right Red Alignment", "%.2f %b", FreightFrenzyPipeline.rightRedAverage, FreightFrenzyPipeline.alignedRedRight);
            redAlignedCount = (FreightFrenzyPipeline.alignedRedLeft ? 1 : 0);
            redAlignedCount += (FreightFrenzyPipeline.alignedRedCenter ? 1 : 0);
            redAlignedCount += (FreightFrenzyPipeline.alignedRedRight ? 1 : 0);
            blueAlignedCount = (FreightFrenzyPipeline.alignedBlueLeft ? 1 : 0);
            blueAlignedCount += (FreightFrenzyPipeline.alignedBlueCenter ? 1 : 0);
            blueAlignedCount += (FreightFrenzyPipeline.alignedBlueRight ? 1 : 0);
            if(redAlignedCount >= 2) {
                telemetry.addLine("Red aligned for red autonomous. Good job!");
            } else if (blueAlignedCount >= 2) {
                telemetry.addLine("WARNING: Blue aligned for RED autonomous. Something is wrong, so so wrong!");
            } else {
                telemetry.addLine("Robot is not aligned for autonomous. Robot so confused!");
            }

            telemetry.update();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Sampling is completed during the INIT stage; No longer need camera active/streaming
        webcam.stopStreaming();
        blockLevel = FreightFrenzyPipeline.blockLevel;
        FreightFrenzyPipeline.saveLastAutoImage();
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
    private void testGyroDrive() {
        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, 24.0, 999.9, DRIVE_THRU ); // Drive FWD 24" along current heading
        gyroDrive(DRIVE_SPEED_30, DRIVE_X, 24.0, 999.9, DRIVE_THRU ); // Strafe RIGHT 24" along current heading
        gyroTurn(TURN_SPEED_20, (getAngle() + 90.0) );       // Turn CW 90 Degrees
    } // testGyroDrive

    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

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

        // Drive to the duck carousel
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "spinDuckCarousel");
            telemetry.update();
            spinDuckCarousel( blockLevel );
        }

        // Drive to square to park
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "driveToSquare");
            telemetry.update();
            driveToSquare( blockLevel );
        }

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void collectTeamElement( int level ) {
        double turnAngle = 0.0;
        double distanceToGrab = 3.2;

        switch( level ) {
            case 3 : turnAngle = 0.0;      // right/top
                     distanceToGrab = -2.0;
                     break;
            case 2 : turnAngle = -25.0;    // middle/middle
                     distanceToGrab = -1.8;
                     break;
            case 1 : turnAngle = -42.0;
                     distanceToGrab = -3.5; // left/bottom
                     break;
        } // switch()

        // Move forward away from field wall so it's safe to raise the arms
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -4.0, 0.0, DRIVE_TO );

        // Rotate the capping arm into the grabbing position
        robot.cappingArmPosition( robot.CAPPING_ARM_POS_GRAB, 0.50 );
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_SPIN, 0.50 );
        sleep( 750);   // wait for capping arm to clear the field wall
        robot.clawServo.setPosition( robot.CLAW_SERVO_OPEN );    // open claw
        robot.wristPositionAuto( robot.WRIST_SERVO_GRAB );       // rotate wrist into the grab position
        robot.boxServo.setPosition( robot.BOX_SERVO_TRANSPORT );
        sleep( 2000);   // wait for arm to reach final position

        // Turn toward the team element
        if( Math.abs(turnAngle) > 0.10 )
            gyroTurn(TURN_SPEED_20, turnAngle );

        // Drive forward to collect the element
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, distanceToGrab, 0.0, DRIVE_TO );
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
        int    freightArmPos = 0;
        long   armSleep = 0;

        switch( level ) {
            case 3 : angleToHub = 40.0;    // top
                     distanceToHub = -8.2;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_TOP_AUTO;
                     armSleep = 0;
                     break;
            case 2 : angleToHub = 38.0;
                     distanceToHub = -4.2;  // middle
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_MIDDLE_AUTO;
                     armSleep = 750;  // 750 msec
                     break;
            case 1 : angleToHub = 35.0;
                     distanceToHub = -5.4;  // bottom
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_BOTTOM_AUTO;
                     armSleep = 1250;   // 1.25 sec
                     break;
        } // switch()

        double currentAngle = robot.headingIMU();

        robot.freightArmPosition( freightArmPos, 0.50 );

        if( Math.abs(angleToHub-currentAngle) > 2.0 )
            gyroTurn(TURN_SPEED_20, angleToHub );

        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, distanceToHub, angleToHub, DRIVE_TO );

        if( armSleep > 0 ) {
            sleep( armSleep );
            gyroDrive(DRIVE_SPEED_30, DRIVE_Y, -3.0, angleToHub, DRIVE_TO );
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

//      robot.sweepServo.setPower( -0.25 );         // start sweeper in reverse
        robot.boxServo.setPosition( servoPos );     // rotate the box to dump
        sleep( 1500 );                    // let cube drop out
//      robot.sweepServo.setPower( 0.0 );           // stop sweeper
        // back away and store arm
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, backDistance, 999.9, DRIVE_TO );
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_TRANSPORT1, 0.50 );
        robot.boxServo.setPosition( robot.BOX_SERVO_COLLECT );
    } // dumpBlock

    /*--------------------------------------------------------------------------------------------*/
    private void spinDuckCarousel( int level ) {
        double towardWall = 0;
        switch( level ) {
            case 3 : towardWall = 17.0; break; // top
            case 2 : towardWall = 18.0; break; // middle
            case 1 : towardWall = 17.0; break; // bottom
        } // switch()
        gyroTurn(TURN_SPEED_20, -90.0 );   // Turn toward wall
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -towardWall, -90.0, DRIVE_TO );
        double wallDistance = backRangeSensor()/2.54 - 10.0;
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -wallDistance, -90.0, DRIVE_TO );
        gyroTurn(TURN_SPEED_20, -135.0 );  // Turn toward corner
        robot.duckMotor.setPower( 0.55 );                // Enable the carousel motor
        // We want to press against the carousel with out trying to reach a given point
        for( int loop=0; loop<5; loop++ ) {
            double barelyPressSpeed = 0.06;
            switch(loop) {
                case 0 : barelyPressSpeed = 0.06; break;
                case 1 : barelyPressSpeed = 0.05; break;
                case 2 : barelyPressSpeed = 0.03; break;
                case 3 : barelyPressSpeed = 0.02; break;
                case 4 : barelyPressSpeed = 0.02; break;
            }
            robot.driveTrainMotors( -barelyPressSpeed, -barelyPressSpeed, -barelyPressSpeed, -barelyPressSpeed );
            sleep( 1000 );   // Spin the carousel for 5 seconds total
        } // loop
        robot.duckMotor.setPower( 0.0 );  // Disable carousel motor
    } // spinDuckCarousel

    /*--------------------------------------------------------------------------------------------*/
    private void driveToSquare( int level ) {
        gyroTurn(TURN_SPEED_20, -180.0 );   // Turn square to side wall
        double squareDistance = 30.0 - backRangeSensor()/2.54;
        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, squareDistance, 999.9, DRIVE_TO );
        // Don't lower arm to floor until we get into the square, in case the freight box has rotated
        // (the front edge will catch on the floor tile when we try to drive forward)
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_COLLECT, 0.20 );
        gyroDrive(DRIVE_SPEED_40, DRIVE_X, -5.0, 999.9, DRIVE_TO );
        // Until Autonomous ends (30 seconds), wait for arm to come down
        while( opModeIsActive() ) {
            sleep(75);  // wait for arm to lower
        }
    } // driveToSquare

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

} /* AutonomousRducks */
