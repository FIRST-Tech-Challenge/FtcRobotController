/* FTC Team 7572 - Version 1.0 (11/13/2021)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name="Autonomous-Blue", group="7592", preselectTeleOp = "Teleop-Blue")
//@Disabled
public class AutonomousB extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBothHubs robot = new HardwareBothHubs();

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y              = true;    // Drive forward/backward
    static final boolean DRIVE_X              = false;   // Drive right/left (not DRIVE_Y)

    static final double  DRIVE_SPEED_10       = 0.10;    // Lower speed for moving from a standstill
    static final double  DRIVE_SPEED_20       = 0.20;    // Lower speed for moving from a standstill
    static final double  DRIVE_SPEED_30       = 0.30;    // Lower speed for fine control going sideways
    static final double  DRIVE_SPEED_40       = 0.40;    // Normally go slower to achieve better accuracy
    static final double  DRIVE_SPEED_55       = 0.55;    // Somewhat longer distances, go a little faster
    static final double  TURN_SPEED_25        = 0.25;    // Nominal half speed for better accuracy.

    static final double  HEADING_THRESHOLD    = 2.0;     // Minimum of 1 degree for an integer gyro
    static final double  P_TURN_COEFF         = 0.050;   // Larger is more responsive, but also less stable
    static final double  P_DRIVE_COEFF        = 0.005;   // Larger is more responsive, but also less stable

    static final int     DRIVE_TO             = 1;       // STOP  after the specified movement 
    static final int     DRIVE_THRU           = 2;       // COAST after the specified movement

    private boolean teamNumber1     = true;  // default to right side (closest to duck carousel)
    private boolean parkInWarehouse = false; // default to park in storage unit (square)

    OpenCvCamera webcam;
    public static int blockLevel = 0;   // dynamic (gets updated every cycle during INIT)

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap);

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
                webcam.setPipeline(new FreightFrenzyPipeline());
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

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for team color/number
        while (!isStarted()) {
            if( gamepad1.left_bumper  ) teamNumber1 = false;   // left side
            if( gamepad1.right_bumper ) teamNumber1 = true;    // right side
            if( gamepad1.triangle )     parkInWarehouse = true;  // park in square (storage unit)
            if( gamepad1.square )       parkInWarehouse = false; // park in warehouse (balls/cubes)
            telemetry.addData("ALLIANCE", "%s", "BLUE");
            telemetry.addData("Start Position (L/R bumper)", "%s", (teamNumber1)? "RIGHT":"LEFT");
            telemetry.addData("Park (square/triangle)", "%s", (parkInWarehouse)? "Warehouse":"Square");
            telemetry.addData("Block Level", "%d", blockLevel );
            telemetry.update();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Sampling is completed during the INIT stage; No longer need camera active/streaming
        webcam.stopStreaming();
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
        gyroTurn(TURN_SPEED_25, (getAngle() + 90.0) );       // Turn CW 90 Degrees
    } // testGyroDrive

    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

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

        // Drive to warehouse
        if( opModeIsActive() ) {
            if( parkInWarehouse ) {
                telemetry.addData("Motion", "driveToWarehouse");
                telemetry.update();
                driveToWarehouse();
            }
            else {
                telemetry.addData("Motion", "driveToSquare");
                telemetry.update();
                driveToSquare( blockLevel );
            }
        }

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void moveToHub( int level ) {
        double distanceToHub = 8.2;

        switch( level ) {
            case 3 : distanceToHub = 9.0; // top
                     break;
            case 2 : distanceToHub = 6.0; // middle
                     break;
            case 1 : distanceToHub = 7.0; // bottom
                     break;
        } // switch()

        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, -4.0, 999.9, DRIVE_THRU );
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_TRANSPORT1, 0.30 );
        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, -3.0, 999.9, DRIVE_THRU );
        robot.boxServo.setPosition( robot.BOX_SERVO_TRANSPORT );
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -3.0, 999.9, DRIVE_THRU );
        gyroTurn(TURN_SPEED_25, (getAngle() - 30.0) );   // Turn CCW 30 Degrees
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_TRANSPORT1, 0.30 );
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -distanceToHub, 999.9, DRIVE_TO );
    } // moveToHub

    /*--------------------------------------------------------------------------------------------*/
    private void dumpBlock( int level ) {
        int armPos = robot.FREIGHT_ARM_POS_HUB_TOP;
        double servoPos = robot.BOX_SERVO_DUMP_TOP;
        long armWait = 2500;  // 2.5 sec
        double backDistance = 3.0;
        double sweepPower = -0.25;

        switch( level ) {
            case 3 : armPos = 1950; // robot.FREIGHT_ARM_POS_HUB_TOP;
                     servoPos = robot.BOX_SERVO_DUMP_TOP;
                     armWait = 3000;
                     backDistance = 4.5;
                     sweepPower = -0.25;
                     break;
            case 2 : armPos = 2275; // robot.FREIGHT_ARM_POS_HUB_MIDDLE;
                     servoPos = robot.BOX_SERVO_DUMP_MIDDLE;
                     armWait = 3600;
                     backDistance = 3.5;
                     sweepPower = -0.30;
                     break;
            case 1 : armPos = 2450; // robot.FREIGHT_ARM_POS_HUB_BOTTOM;
                     servoPos = robot.BOX_SERVO_DUMP_BOTTOM;
                     armWait = 4800;
                     backDistance = 3.5;
                     sweepPower = -0.50;
                     break;
        } // switch()

        robot.freightArmPosition( armPos,0.20 );             // move the arm down
        sleep( armWait );                                    // let arm get to position
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -1.6, 999.9, DRIVE_TO );
        robot.sweepServo.setPower( sweepPower );             // start sweeper in reverse
        robot.boxServo.setPosition( servoPos );              // rotate the box to dump
        sleep( 1500 );                                       // let cube drop out
        robot.sweepServo.setPower( 0.0 );                    // stop sweeper
        // back away and store arm
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, backDistance, 999.9, DRIVE_TO );
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_TRANSPORT1, 0.50 );
    } // dumpBlock

    /*--------------------------------------------------------------------------------------------*/
    private void spinDuckCarousel( int level ) {
        double fwdDistance = 28.0;
        double turnAngle = 140.0;

        switch( level ) {
            case 3 : fwdDistance = 28.0; // top
                     turnAngle = 140.0;
                     break;
            case 2 : fwdDistance = 28.0; // middle
                     turnAngle = 140.0;
                     break;
            case 1 : fwdDistance = 28.0; // bottom
                     turnAngle = 140.0;
                     break;
        } // switch()
        
        double barelyPressSpeed = 0.07;
        gyroTurn(TURN_SPEED_25, (getAngle() + turnAngle) );  // Turn toward corner
        robot.freightArmPosition( robot.FREIGHT_ARM_POS_COLLECT, 0.20 );
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -fwdDistance, 999.9, DRIVE_TO );
        robot.duckMotor.setPower( -0.64 );               // Enable the carousel motor
        gyroTurn(TURN_SPEED_25, (getAngle() + 16.0) );   // Turn more (into carousel table)
        // We want to press against the carousel with out trying to reach a given point
        robot.driveTrainMotors( -barelyPressSpeed, -barelyPressSpeed, -barelyPressSpeed, -barelyPressSpeed );
        sleep( 6000 );                         // Spin the carousel for 6 seconds
        robot.duckMotor.setPower( 0.0 );                 // Disable carousel motor
    } // spinDuckCarousel

   /*--------------------------------------------------------------------------------------------*/
    private void driveToWarehouse() {
/*
        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, 7.0, 999.9, DRIVE_THRU );
        gyroTurn(TURN_SPEED_25, (getAngle() - 20.0) );   // Turn square to front wall
        gyroDrive(DRIVE_SPEED_40, DRIVE_Y, 20.0, 999.9, DRIVE_THRU );

        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, 20.0, 999.9, DRIVE_TO );
//      robot.freightArmPosition( robot.FREIGHT_ARM_POS_COLLECT, 0.20 );
*/
    } // driveToWarehouse

    /*--------------------------------------------------------------------------------------------*/
    private void driveToSquare( int level ) {
        double fwdDistance = 19.0;
        double turnAngle = 52.0;

        switch( level ) {
            case 3 : fwdDistance = 18.0; // top
                     turnAngle = 57.0;
                     break;
            case 2 : fwdDistance = 19.0; // middle
                     turnAngle = 52.0;
                     break;
            case 1 : fwdDistance = 18.0; // bottom
                     turnAngle = 52.0;
                     break;
        } // switch()
        gyroDrive(DRIVE_SPEED_30, DRIVE_X, -5.5, 999.9, DRIVE_THRU );
        gyroTurn(TURN_SPEED_25, (getAngle() + turnAngle) );   // Turn square to front wall
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, fwdDistance, 999.9, DRIVE_TO );
    } // driveToSquare

    /*---------------------------------------------------------------------------------------------
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive( double maxSpeed, boolean driveY, double distance, double angle, int driveType ) {
        double  error, steer;
        double  curSpeed, leftSpeed, rightSpeed, max;
        boolean reachedTarget;
        int     loopCount = 0;
        int     posTol = (driveType == DRIVE_TO)? 15:40;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();

            // range check the provided angle
            if( (angle > 360.0) || (angle < -360.0) ) {
                angle = getAngle();  // maintain current angle
            }

            // configure RUN_TO_POSITION drivetrain motor setpoints
            robot.setRunToPosition( driveY, distance );

            // start motion.
            maxSpeed = Range.clip(Math.abs(maxSpeed), -1.0, 1.0);
            robot.frontLeftMotor.setPower(maxSpeed);
            robot.frontRightMotor.setPower(maxSpeed);
            robot.rearLeftMotor.setPower(maxSpeed);
            robot.rearRightMotor.setPower(maxSpeed);

            // Allow movement to start before checking isBusy()
            sleep( 150 ); // 150 msec

            // keep looping while we are still active, and ALL motors are running.
            while( opModeIsActive() ) {

                // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
                robot.readBulkData();

                int frontLeftError  = Math.abs(robot.frontLeftMotorTgt - robot.frontLeftMotorPos);
                int frontRightError = Math.abs(robot.frontRightMotorTgt - robot.frontRightMotorPos);
                int rearLeftError   = Math.abs(robot.rearLeftMotorTgt - robot.rearLeftMotorPos);
                int rearRightError  = Math.abs(robot.rearRightMotorTgt - robot.rearRightMotorPos);
                int avgError = (frontLeftError + frontRightError + rearLeftError + rearRightError) / 4;

                // 19.2:1 is 537 counts/rotation (18.85" distance).  1/2" tolerance = 14.24 counts
                reachedTarget = ((frontLeftError < posTol) && (frontRightError < posTol) &&
                                 (rearLeftError  < posTol) && (rearRightError  < posTol) );
//              reachedTarget = !robot.frontLeftMotor.isBusy() && !robot.frontRightMotor.isBusy() &&
//                              !robot.rearLeftMotor.isBusy() && !robot.rearRightMotor.isBusy();
                if (reachedTarget) break;

                // update range sensor data
//              robot.updateRangeL();
//              robot.updateRangeR();

                // reduce motor speed as we get close so we don't overshoot
                if (avgError < 75) {
                    curSpeed = 0.12;
                } else if (avgError < 300) {
                    curSpeed = 0.67 * maxSpeed;
                }
                else {
                    curSpeed = maxSpeed;
                }

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed  = curSpeed + steer;
                rightSpeed = curSpeed - steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.frontLeftMotor.setPower(leftSpeed);
                robot.frontRightMotor.setPower(rightSpeed);
                robot.rearLeftMotor.setPower(leftSpeed);
                robot.rearRightMotor.setPower(rightSpeed);

                loopCount++; // still working, or stale data?

                // Display drive status for the driver.
                if( true ) {
                    telemetry.addData("loopCount", "%d", loopCount );
                    telemetry.addData("Err/St", "%5.1f/%5.3f", error, steer);
                    telemetry.addData("Target F", "%7d:%7d", robot.frontLeftMotorTgt, robot.frontRightMotorTgt);
                    telemetry.addData("Target R", "%7d:%7d", robot.rearLeftMotorTgt, robot.rearRightMotorTgt);
                    telemetry.addData("Error F", "%7d:%7d", frontLeftError, frontRightError);
                    telemetry.addData("Error R", "%7d:%7d", rearLeftError, rearRightError);
                    telemetry.addData("Speed", "%5.3f:%5.3f", leftSpeed, rightSpeed);
                    telemetry.update();
                }
            } // opModeIsActive

            if( driveType == DRIVE_TO ) {
                // Stop all motion;
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.rearLeftMotor.setPower(0);
                robot.rearRightMotor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
//          sleep( 100 ); // 100 msec delay before changing modes
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } // opModeIsActive()
    } // gyroDrive()

    /*---------------------------------------------------------------------------------------------
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn( double speed, double angle ) {

        // keep looping while we are still active, and not on heading.
        while( opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, 0.0,0.0) ) {
            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();

            // update range sensor data
//          robot.updateRangeL();
//          robot.updateRangeR();
        }
    } // gyroTurn()

    /*---------------------------------------------------------------------------------------------
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // range check the provided angle
        if( (angle > 360.0) || (angle < -360.0) ) {
            angle = getAngle();  // maintain current angle
        }

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF, 0.0,0.0);
//          telemetry.update();
        }

        // Stop all motion;
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
    } // gyroHold

    /*---------------------------------------------------------------------------------------------
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @param left_right Movement left/right  (positive=left,    negative=right)
     * @param fore_aft   Movement forward/aft (positive=forward, negative=aft)
     * @return onHeading
     */
    boolean onHeading(double speed, double angle, double PCoeff, double left_right, double fore_aft ) {
        double  error;
        double  steer;
        double  baseSpeed;
        double  faSpeed;
        double  lrSpeed;
        boolean onTarget = false;
        double  frontLeft;
        double  frontRight;
        double  backLeft;
        double  backRight;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            frontLeft  = 0.0;  // desired angle achieved; shut down all 4 motors
            frontRight = 0.0;
            backLeft   = 0.0;
            backRight  = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            baseSpeed  = speed * steer;
            lrSpeed    = speed * left_right;  // scale left/right speed (-1.0 .. +1.0) using turn speed
            faSpeed    = speed * fore_aft;    // scale fore/aft   speed (-1.0 .. +1.0) using turn speed
            frontLeft  =  baseSpeed + lrSpeed;  // increases LEFT speed
            frontRight = -baseSpeed + lrSpeed;  // decreased RIGHT speed (less negative)
            backLeft   = frontLeft  - faSpeed;  // decreases
            backRight  = frontRight + faSpeed;  // increases
        }

        // Send desired speeds to motors.
        robot.frontLeftMotor.setPower(frontLeft);
        robot.frontRightMotor.setPower(frontRight);
        robot.rearLeftMotor.setPower(backLeft);
        robot.rearRightMotor.setPower(backRight);

        // Display drive status for the driver.
        if( true ) {
            // Display it for the driver.
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed Front", "%5.2f:%5.2f", frontLeft, frontRight);
            telemetry.addData("Speed Back ", "%5.2f:%5.2f", backLeft, backRight);
            telemetry.update();
        }
        return onTarget;
    } // onHeading()

    /*---------------------------------------------------------------------------------------------
     * getAngle queries the current gyro angle
     * @return  current gyro angle (-179.9 to +180.0)
     */
    private double getAngle() {
        // calculate error in -179.99 to +180.00 range  (
        double gryoAngle = robot.headingIMU();
        while (gryoAngle >   180.0) gryoAngle -= 360.0;
        while (gryoAngle <= -180.0) gryoAngle += 360.0;
        return gryoAngle;
    } // getAngle()

    /*---------------------------------------------------------------------------------------------
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          Positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {
        // calculate error in -179 to +180 range  (
        double robotError = targetAngle - robot.headingIMU();
        while (robotError >  180.0)  robotError -= 360.0;
        while (robotError <= -180.0) robotError += 360.0;
        return robotError;
    } // getError()

    /*---------------------------------------------------------------------------------------------
     * returns desired steering force.  +/- 1 range.  positive = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return clippedSteering
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    } // getSteer()

    /* Skystone image procesing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class FreightFrenzyPipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        private Mat YCrCb = new Mat();
        private Mat Cb    = new Mat();
        private Mat subMat1;
        private Mat subMat2;
        private Mat subMat3;
        private int max;
        private int avg1;
        private int avg2;
        private int avg3;
        private Point skystone   = new Point();        // Team Element (populated once we find it!)
        private Point sub1PointA = new Point(18, 190);  // 15x15 pixels on LEFT stone
        private Point sub1PointB = new Point(33, 205);
        private Point sub2PointA = new Point(151, 190); // 15x15 pixels on CENTER stone (col 160)
        private Point sub2PointB = new Point(167, 205);
        private Point sub3PointA = new Point(270, 190); // 15x15 pixels on RIGHT stone
        private Point sub3PointB = new Point(285, 205);

        @Override
        public Mat processFrame(Mat input)
        {
            // Convert image frame from RGB to YCrCb
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            // Extract the Cb channel from the image frame
            Core.extractChannel(YCrCb, Cb, 2);
            // Pull data for the three sample zones from the Cb channel
            subMat1 = Cb.submat(new Rect(sub1PointA,sub1PointB) );
            subMat2 = Cb.submat(new Rect(sub2PointA,sub2PointB) );
            subMat3 = Cb.submat(new Rect(sub3PointA,sub3PointB) );
            // Average the three sample zones
            avg1 = (int)Core.mean(subMat1).val[0];
            avg2 = (int)Core.mean(subMat2).val[0];
            avg3 = (int)Core.mean(subMat3).val[0];
            // Draw rectangles around the sample zones
            Imgproc.rectangle(input, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, sub2PointA, sub2PointB, new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, sub3PointA, sub3PointB, new Scalar(0, 0, 255), 1);
            // Determine which sample zone had the lowest contrast from blue (lightest color)
            max = Math.min(avg1, Math.min(avg2, avg3));
            // Draw a circle on the detected skystone
            if(max == avg1) {
                skystone.x = (sub1PointA.x + sub1PointB.x) / 2;
                skystone.y = (sub1PointA.y + sub1PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                blockLevel = 1;
            } else if(max == avg2) {
                skystone.x = (sub2PointA.x + sub2PointB.x) / 2;
                skystone.y = (sub2PointA.y + sub2PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                blockLevel = 2;
            } else if(max == avg3) {
                skystone.x = (sub3PointA.x + sub3PointB.x) / 2;
                skystone.y = (sub3PointA.y + sub3PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                blockLevel = 3;
            } else {
                blockLevel = 3;
            }

            // Free the allocated submat memory
            subMat1.release();
            subMat1 = null;
            subMat2.release();
            subMat2 = null;
            subMat3.release();
            subMat3 = null;

            return input;
        }
    } // FreightFrenzyPipeline

} /* AutonomousB */
