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

package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain_v3;

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

@Autonomous(name="Pushbot: Auto Drive By BNO055 IMU_v4", group="Pushbot")
//@Disabled
public class BNO055_mod_opmode_v4 extends LinearOpMode {

    /* Declare OpMode members. */
    private Drivetrain drivetrain   = new Drivetrain(true);   // Use subsystem Drivetrain
    private Orientation     lastAngles  = new Orientation();
    private ElapsedTime     PIDtimer    = new ElapsedTime(); // PID loop timer
    private ElapsedTime     runtime     = new ElapsedTime(); // timeout timer

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.6;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.50;    // 0.4 for berber carpet. Check on mat too

    static final double     HEADING_THRESHOLD       = 2;      // As tight as we can make it with an integer gyro
    static final double     Kp_TURN                 = 0.0275;   //0.025 to 0.0275 on mat seems to work
    static final double     Ki_TURN                 = 0.003;   //0.0025 to 0.004 on a mat works. Battery voltage matters
    static final double     Kd_TURN                 = 0.0;   //leave as 0
    static final double     Kp_DRIVE                = 0.05;   //0.05 Larger is more responsive, but also less stable
    static final double     Ki_DRIVE                = 0.005;   // 0.005 Larger is more responsive, but also less stable
    static final double     Kd_DRIVE                = 0.0;   // Leave as 0 for now


    double                  globalAngle; // not used currently
    double                  lasterror;
    double                  totalError;

    @Override
    public void runOpMode() {

        drivetrain.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        drivetrain.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivetrain.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Calibrate
        drivetrain.imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && drivetrain.imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        drivetrain.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", drivetrain.imu.getCalibrationStatus().toString());
        telemetry.update();

       /////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 80.0, 0.0, 5);    // Drive FWD 110 inches
        gyroDrive(DRIVE_SPEED, -80.0, 0.0, 5);    // Drive FWD 110 inches

        gyroDrive(DRIVE_SPEED, 20.0, 0.0, 5);    // Drive FWD 110 inches
        gyroTurn( TURN_SPEED, 90.0, 3);         // Turn  CCW to -45 Degrees
        //gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        gyroDrive(DRIVE_SPEED, 20.0, 90, 5);  // Drive FWD 12 inches at 45 degrees
        gyroTurn( TURN_SPEED,  180.0,4);         // Turn  CW  to  45 Degrees
        //gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        //gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        //gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED,20.0, 180, 3);    // Drive REV 48 inches
        gyroTurn( TURN_SPEED,  270.0,3);         // Turn  CW  to  45 Degrees

        gyroDrive(DRIVE_SPEED,20.0, 270.0, 5);    // D rive REV 48 inches
        gyroTurn( TURN_SPEED,  0,3);         // Turn  CW  to  45 Degrees
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *  3) Timeout time is reached - prevents robot from getting stuck
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle, double timeout) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        totalError = 0;
        lasterror = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive() & runtime.time() < timeout) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * Drivetrain.COUNTS_PER_INCH);
            newLeftTarget = drivetrain.leftFront.getCurrentPosition() + moveCounts;
            newRightTarget = drivetrain.rightFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
           drivetrain.leftFront.setTargetPosition(newLeftTarget);
           drivetrain.rightFront.setTargetPosition(newRightTarget);

           drivetrain.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           drivetrain.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
           drivetrain.leftFront.setPower(speed);
           drivetrain.rightFront.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (drivetrain.leftFront.isBusy() && drivetrain.rightFront.isBusy())) {

                // adjust relative speed based on heading error.
                // Positive angle means drifting to the left so need to steer to the
                // right to get back on track.
                error = getError(angle);
                steer = getSteer(error, Kp_DRIVE, Ki_DRIVE, Kd_DRIVE);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                drivetrain.leftFront.setPower(leftSpeed);
                drivetrain.rightFront.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      drivetrain.leftFront.getCurrentPosition(),
                                                             drivetrain.rightFront.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();


            }

            // Stop all motion;
            drivetrain.leftFront.setPower(0);
            drivetrain.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            drivetrain.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drivetrain.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
            runtime.reset(); // reset the timer for the next function call
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *  3) Timeout time has elapsed - prevents getting stuck
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param timeout max time allotted to complete each call to gyroTurn
     */
    public void gyroTurn (  double speed, double angle, double timeout) {
        totalError = 0;
        lasterror = 0;
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, Kp_TURN, Ki_TURN, Kd_TURN) && runtime.time() < timeout) {
            // Update telemetry & Allow time for other processes to run.
            //onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }
        runtime.reset(); // reset after we are done with the while loop
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
            onHeading(speed, angle, Kp_TURN, Ki_TURN, Kd_TURN);
            telemetry.update();
        }

        // Stop all motion;
        drivetrain.leftFront.setPower(0);
        drivetrain.rightFront.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     *
     * @return
     */
    boolean onHeading(double speed, double angle, double P_TURN_COEFF , double I_TURN_COEFF, double D_TURN_COEFF) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, P_TURN_COEFF , I_TURN_COEFF, D_TURN_COEFF);
            rightSpeed  = -speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        drivetrain.leftFront.setPower(leftSpeed);
        drivetrain.rightFront.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     *
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        // instantiate an angles object from the IMU
        Orientation angles = drivetrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // pull out the first angle which is the Z axis for heading and use to calculate the error
        // Positive robot rotation is left so positive error means robot needs to turn right.
        robotError = angles.firstAngle - targetAngle; //lastAngles.firstAngle;
        telemetry.addData("Robot Error", robotError);
        telemetry.addData("Target Angle", targetAngle);
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @param ICoef Integration coefficient to apply to the area under the error-time curve
     * @param DCoef Derivative coefficient to apply to the area under the error-time curve
     * @return
     */
    public double getSteer(double error, double PCoeff, double ICoef, double DCoef) {
        double P; // combined proportional error Kp*error
        double I; // combined integral error Ki * cumulative error
        double D; // combined derivative error Kd*change in error
        double changeInError;

        changeInError = error - lasterror;
        P = PCoeff * error;
        totalError = totalError  + error * PIDtimer.time();
        I = ICoef * totalError;
        D = DCoef * (changeInError)/PIDtimer.time();
        lasterror = error;
        PIDtimer.reset();

        //return Range.clip(error *(PCoeff+ICoef+DCoef), -1, 1);
        return Range.clip((P+I+D),-1,1);

    }

}
