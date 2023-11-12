/* FTC Team 7572 - Version 1.0 (11/11/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

/**
 * Unit Test program to verify that odometry encoder counts have correct sign.
 * Left/Right should be POSITIVE when robot is pushed forward.
 * Rear or strafe horiztonal encounter count should be POSITIVE when robot is pushed right.
 * Assumes OdometryCalibration program has already been run to create the constants files.
 * Can also be used to manually tweak the odometry constants based on field measurements.
 */
@TeleOp(name = "Odometry Manual Cal", group = "Calibrate")
@Disabled
public class CalibrationOdometryManual extends LinearOpMode {
    double  yTranslation, xTranslation, rotation;                  /* Driver control inputs */
    double  backLeft, backRight, frontLeft, frontRight, maxPower;  /* Motor power levels */
    long    nanoTimeCurr=0, nanoTimePrev=0;
    double  elapsedTime, elapsedHz;

    /* Declare OpMode members. */
    HardwarePixelbot robot = new HardwarePixelbot();

    //Files to access the algorithm constants
    File wheelBaseSeparationFile  = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    double robotEncoderWheelDistance            = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * robot.COUNTS_PER_INCH2;
    double horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    double robotGlobalXCoordinatePosition       = 0.0;   // in odometer counts
    double robotGlobalYCoordinatePosition       = 0.0;
    double robotOrientationRadians              = 0.0;   // 0deg (straight forward)
    double imuAngleInitial                      = 0.0;   // (for comparison purposes -- not completely accurate)
    double imuAngleCurrent                      = 0.0;
    double imuAngleDegrees                      = 0.0;

    double robotEncoderWheelDistanceError;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot hardware
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();
        robot.init(hardwareMap,false);

        // Adjust odometry constants based on field measurements
        // (auto-calibration used the IMU, which isn't perfect)
        robotEncoderWheelDistance            -= 0.000;
        horizontalEncoderTickPerDegreeOffset += 0.000;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "Ready");
        telemetry.update();
        waitForStart();

        // Bulk-query all odometry data (establish starting position)
//      robot.resetOdometryEncoders();
        robot.readBulkData();
        imuAngleInitial = robot.headingIMU();

        // run until the end of the match (driver presses STOP)
        while( opModeIsActive() )
        {
            // Bulk-query all odometry data (delta since last reading)
            robot.readBulkData();

            // Compute updated robot position/orientation
            globalCoordinatePositionUpdate();
            imuAngleCurrent = robot.headingIMU();
            imuAngleDegrees = AngleWrapDegrees( imuAngleCurrent - imuAngleInitial );

            processStandardDriveMode();

            // NOTE: By starting the robot aligned on the corner of a field tile, and then
            // manually rotating 10 full circles (ending up aligned with the floor tile edge)
            // an error-factor can be computed for the current robotEncoderWheelDistance value.
            // This value must be divided by robot.COUNTS_PER_INCH2 to view it in INCHES.
            robotEncoderWheelDistanceError = Math.abs(robot.leftOdometerCount) + Math.abs(robot.rightOdometerCount);
            robotEncoderWheelDistanceError /= (10 * 2*Math.PI);   // the expected value for 10 loops
            robotEncoderWheelDistanceError -= robotEncoderWheelDistance;

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            elapsedTime  = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;       // msec
            elapsedHz    = (elapsedTime > 0.0)? (1000.0/elapsedTime): 0.0; // Hz

            // Update telemetry data
            telemetry.addData("Wheel Base Separation", (robotEncoderWheelDistance / robot.COUNTS_PER_INCH2) );
            telemetry.addData("Wheel Base Error (10 rotations)", robotEncoderWheelDistanceError );
            telemetry.addData("Horizontal Tick/Degree", horizontalEncoderTickPerDegreeOffset );
            telemetry.addData("Odometry left",   "%d", robot.leftOdometerCount  );
            telemetry.addData("Odometry right",  "%d", robot.rightOdometerCount );
            telemetry.addData("Odometry strafe", "%d", robot.strafeOdometerCount );
            telemetry.addData("World X",     "%.2f in", (robotGlobalYCoordinatePosition / robot.COUNTS_PER_INCH2) );
            telemetry.addData("World Y",     "%.2f in", (robotGlobalXCoordinatePosition / robot.COUNTS_PER_INCH2) );
            telemetry.addData("Orientation", "%.2f deg (IMU %.2f)", Math.toDegrees(robotOrientationRadians), imuAngleDegrees );
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", elapsedTime, elapsedHz );
            telemetry.update();
        } // opModeIsActive

    } // runOpMode()

    private double multSegLinearRot( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.02 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.33 ) {                      // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.0650;   // 0.02=0.070  0.33=0.1475
            }
            else if( valueIn < 0.60 ) {
                valueOut = (0.50 * valueIn) - 0.0175;   // 0.33=0.1475  0.60=0.2825
            }
            else if( valueIn < 0.90 ) {
                valueOut = (0.75 * valueIn) - 0.1675;   // 0.60=0.2825  0.90=0.5075
            }
            else
                valueOut = (6.00 * valueIn) - 4.8925;   // 0.90=0.5075  1.00=1.1075 (clipped!)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.33 ) {
                valueOut = (0.25 * valueIn) - 0.0650;
            }
            else if( valueIn > -0.60 ) {
                valueOut = (0.50 * valueIn) + 0.0175;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (0.75 * valueIn) + 0.1675;
            }
            else
                valueOut = (6.00 * valueIn) + 4.8925;
        }

        return valueOut;
    } // multSegLinearRot

    private double multSegLinearXY( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.02 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.33 ) {                       // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.0550;    // 0.01=0.060   0.33=0.1375
            }
            else if( valueIn < 0.90 ) {
                valueOut = (1.00 * valueIn) - 0.1925;   // 0.33=0.1375   0.90=0.7075
            }
            else
                valueOut = (14.0 * valueIn) - 11.8925;  // 0.90=0.7075   1.00=2.1075 (clipped)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.33 ) {
                valueOut = (0.25 * valueIn) - 0.0550;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (1.00 * valueIn) + 0.1925;
            }
            else
                valueOut = (14.0 * valueIn) + 11.8925;
        }

        return valueOut;
    } // multSegLinearXY

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Standard Mecanum-wheel drive control (no dependence on gyro!)         */
    /*---------------------------------------------------------------------------------*/
    void processStandardDriveMode() {
        // Retrieve X/Y and ROTATION joystick input
        yTranslation = multSegLinearXY( -gamepad1.left_stick_y );
        xTranslation = multSegLinearXY(  gamepad1.left_stick_x );
        rotation     = multSegLinearRot( -gamepad1.right_stick_x );
        // Normal teleop drive control:
        // - left joystick is TRANSLATE fwd/back/left/right
        // - right joystick is ROTATE clockwise/counterclockwise
        // NOTE: assumes the right motors are defined FORWARD and the
        // left motors are defined REVERSE so positive power is FORWARD.
        frontRight = yTranslation - xTranslation + rotation;
        frontLeft  = yTranslation + xTranslation - rotation;
        backRight  = yTranslation + xTranslation + rotation;
        backLeft   = yTranslation - xTranslation - rotation;
        // Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(backLeft),  Math.abs(backRight)  ),
                Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if (maxPower > 1.0)
        {
            backLeft   /= maxPower;
            backRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }
        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, backLeft, backRight );
    } // processStandardDriveMode


    /**
     * Ensure angle is in the range of -180 to +180 deg (-PI to +PI)
     * @param angleDegrees
     * @return
     */
    public double AngleWrapDegrees( double angleDegrees ){
        while( angleDegrees < -180 ) {
            angleDegrees += 360.0;
        }
        while( angleDegrees > 180 ){
            angleDegrees -= 360.0;
        }
        return angleDegrees;
    } // AngleWrapDegrees

    /**
     * Ensure angle is in the range of -PI to +PI (-180 to +180 deg)
     * @param angleRadians
     * @return
     */
    public double AngleWrapRadians( double angleRadians ){
        while( angleRadians < -Math.PI ) {
            angleRadians += 2.0*Math.PI;
        }
        while( angleRadians > Math.PI ){
            angleRadians -= 2.0*Math.PI;
        }
        return angleRadians;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        int leftChange  = robot.leftOdometerCount  - robot.leftOdometerPrev;
        int rightChange = robot.rightOdometerCount - robot.rightOdometerPrev;
        //Calculate Angle
        double changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians += changeInRobotOrientation;
        robotOrientationRadians = AngleWrapRadians( robotOrientationRadians );   // Keep between -PI and +PI
        //Get the components of the motion
        int rawHorizontalChange = robot.strafeOdometerCount - robot.strafeOdometerPrev;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);
        double p = ((rightChange + leftChange) / 2.0);
        double n = horizontalChange;
        //Calculate and update the position values
        robotGlobalXCoordinatePosition += (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition += (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));
    } // globalCoordinatePositionUpdate

} // CalibrationOdometryManual
