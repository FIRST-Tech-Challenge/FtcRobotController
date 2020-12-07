/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SubSystems.ChassisClassic;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepadClassic;

//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="TestOpMode : Pushbot: Auto Drive By IMU Gyro", group="TestOpMode")
@Disabled
public class PushbotAutoDriveByGyro_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    //ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    HzGamepadClassic hzGamepad;
    ChassisClassic hzChassisClassic;

    // The IMU sensor object
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;//, power = .30, correction, rotation;

    //static final double     COUNTS_PER_MOTOR_REV    = //1440 ;    // eg: TETRIX Motor Encoder
    //static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    //static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference

    static final double     COUNTS_PER_MOTOR_REV    =537.6;  // for 5202 19.2:1 Ratio, 312 RPM
                                                            // 383.6 for 5202 13.7:1 Ratio, 435 RPM motor
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // 96mm For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;//0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;//0.15;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        //robot.init(hardwareMap);
        hzChassisClassic = new ChassisClassic(hardwareMap);
        hzGamepad = new HzGamepadClassic(gamepad1, this);


        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        //robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        //gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        //while (!isStopRequested() && gyro.isCalibrating())  {
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        //robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // wait for start button.

        waitForStart();

        hzChassisClassic.initChassis();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);


    // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            //telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            getAngle();
            telemetry.addData(">", "Robot Heading = %d", lastAngles.firstAngle);
            telemetry.update();
        }

        resetAngle();

        //gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
        gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
        gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches
        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

   /**
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
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        //int     newLeftTarget;
        //int     newRightTarget;
        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;


        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  frontLeftSpeed;
        double  frontRightSpeed;
        double  backLeftSpeed;
        double  backRightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            //newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            //newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            newFrontLeftTarget = hzChassisClassic.frontLeft.getCurrentPosition() + moveCounts;
            newFrontRightTarget = hzChassisClassic.frontRight.getCurrentPosition() + moveCounts;
            newBackLeftTarget = hzChassisClassic.backLeft.getCurrentPosition() + moveCounts;
            newBackRightTarget = hzChassisClassic.backRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            //robot.leftDrive.setTargetPosition(newLeftTarget);
            //robot.rightDrive.setTargetPosition(newRightTarget);

            hzChassisClassic.frontLeft.setTargetPosition(newFrontLeftTarget);
            hzChassisClassic.frontRight.setTargetPosition(newFrontRightTarget);
            hzChassisClassic.backLeft.setTargetPosition(newBackLeftTarget);
            hzChassisClassic.backRight.setTargetPosition(newBackRightTarget);

            //robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            hzChassisClassic.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hzChassisClassic.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hzChassisClassic.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hzChassisClassic.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            //robot.leftDrive.setPower(speed);
            //robot.rightDrive.setPower(speed);

            hzChassisClassic.frontLeft.setPower(speed);
            hzChassisClassic.frontRight.setPower(speed);
            hzChassisClassic.backLeft.setPower(speed);
            hzChassisClassic.backRight.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   //(robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
                   (hzChassisClassic.frontLeft.isBusy() && hzChassisClassic.frontRight.isBusy() &&
                    hzChassisClassic.backLeft.isBusy() && hzChassisClassic.backRight.isBusy())) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                //max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                max = Math.max( Math.abs(leftSpeed), Math.abs(rightSpeed));

                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                //robot.leftDrive.setPower(leftSpeed);
                //robot.rightDrive.setPower(rightSpeed);

                hzChassisClassic.frontLeft.setPower(leftSpeed);
                hzChassisClassic.frontRight.setPower(rightSpeed);
                hzChassisClassic.backLeft.setPower(leftSpeed);
                hzChassisClassic.backRight.setPower(rightSpeed);


                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                //telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                //telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());

                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontLeftTarget,newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      hzChassisClassic.frontLeft.getCurrentPosition(),
                        hzChassisClassic.frontRight.getCurrentPosition(), hzChassisClassic.backLeft.getCurrentPosition(), hzChassisClassic.backRight.getCurrentPosition());


                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            //robot.leftDrive.setPower(0);
            //robot.rightDrive.setPower(0);

            hzChassisClassic.frontLeft.setPower(0);
            hzChassisClassic.frontRight.setPower(0);
            hzChassisClassic.backLeft.setPower(0);
            hzChassisClassic.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            //robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hzChassisClassic.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hzChassisClassic.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hzChassisClassic.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hzChassisClassic.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        getAngle();
    }

    /**
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
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        getAngle();
    }

    /**
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

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        //robot.leftDrive.setPower(0);
        //robot.rightDrive.setPower(0);

        hzChassisClassic.frontLeft.setPower(0);
        hzChassisClassic.frontRight.setPower(0);
        hzChassisClassic.backLeft.setPower(0);
        hzChassisClassic.backRight.setPower(0);

        getAngle();
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        //double leftSpeed;
        //double rightSpeed;
        double turnSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            //leftSpeed  = 0.0;
            //rightSpeed = 0.0;
            turnSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            //rightSpeed  = speed * steer;
            //leftSpeed   = -rightSpeed;
            turnSpeed = speed * steer;
        }

        // Send desired speeds to motors.
        //robot.leftDrive.setPower(leftSpeed);
        //robot.rightDrive.setPower(rightSpeed);

        hzChassisClassic.frontLeft.setPower(-turnSpeed);
        hzChassisClassic.frontRight.setPower(turnSpeed);
        hzChassisClassic.backLeft.setPower(-turnSpeed);
        hzChassisClassic.backRight.setPower(turnSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        //telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        telemetry.addData("Speed.", "%5.2f", turnSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntegratedZValue();
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
