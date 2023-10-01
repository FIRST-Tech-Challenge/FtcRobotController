package org.firstinspires.ftc.teampractice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RobotDriver {
    /* Declare OpMode members. */
    private final Robot _robot;
    private final ElapsedTime _runtime = new ElapsedTime();
    private final LinearOpMode _opMode;

    private static final double P_TURN_COEFF = 0.05d;     // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.09d;     // Larger is more responsive, but also less stable
    private static final double HEADING_THRESHOLD = 0.36d;      // As tight as we can make it with an integer gyro

    RobotDriver(Robot robot, LinearOpMode opMode) {
        _robot = robot;
        _opMode = opMode;

        _robot.resetDrive();
    }

    public final void gyroSlide(double speed,
                                double distance,
                                double angle,
                                double timeoutS,
                                IObjectDetector<Boolean> objectDetector) {

        // Ensure that the opmode is still active
        if (_opMode.opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            _robot.setDriveTarget(distance, true);
            _robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            double counter = 0d;
            _runtime.reset();
            double powerToSet = Math.max(speed, 0.2d);
            _robot.setDriveVelocity(powerToSet, powerToSet, powerToSet, powerToSet);
            ElapsedTime driveTime = new ElapsedTime();

            // keep looping while we are still active, and BOTH motors are running.
            while (_opMode.opModeIsActive() &&
                    _runtime.seconds() < timeoutS &&
                    _robot.isDriveBusy()) {
                _opMode.idle();

                if (objectDetector != null && objectDetector.objectDetected()) {
                    _robot.beep();
                    break;
                }

                _opMode.telemetry.update();
            }

            // Stop all motion;
            _robot.setDrivePower(0d, 0d, 0d, 0d);

            // Turn off RUN_TO_POSITION
            _robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public final boolean gyroDrive(double speed,
                                   double distance,
                                   double angle,
                                   double timeoutS,
                                   IObjectDetector<Boolean> objectDetector) {

        boolean successful = true;
        // Ensure that the opmode is still active
        if (_opMode.opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            _robot.setDriveTarget(distance, false);
            _robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            _runtime.reset();
            _robot.setDriveVelocity(speed, speed, speed, speed);
            // keep looping while we are still active, and BOTH motors are running.
            while (_opMode.opModeIsActive() &&
                    _runtime.seconds() < timeoutS &&
                    _robot.isDriveBusy()) {

                _opMode.idle();

                // adjust relative speed based on heading error.
                double error = getError(angle);
                double steer;
                if (Math.abs(speed) < 0.2d) {
                    steer = getSteer(error, P_DRIVE_COEFF, Math.abs(speed));
                } else {
                    steer = getSteer(error, P_DRIVE_COEFF);
                }

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0d)
                    steer *= -1d;

                double speedLeftFront, speedLeftRear, speedRightFront, speedRightRear;

                speedLeftFront = speedLeftRear = speed - steer;
                speedRightFront = speedRightRear = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(speedLeftFront), Math.abs(speedRightFront));
                if (max > 1d) {
                    speedLeftFront /= max;
                    speedLeftRear /= max;
                    speedRightFront /= max;
                    speedRightRear /= max;
                }
                if (objectDetector != null && objectDetector.objectDetected()) {
                    _robot.beep();
                    successful = false;
                    break;
                }
                _robot.setDriveVelocity(speedLeftFront,
                        speedLeftRear,
                        speedRightFront,
                        speedRightRear);

                _opMode.telemetry.update();
            }

            // Stop all motion;
            _robot.setDrivePower(0d, 0d, 0d, 0d);

            // Turn off RUN_TO_POSITION
            _robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return successful;
    }

    public final void gyroTurn(double speed, double angle, double timeoutS) {

        _runtime.reset();
        // keep looping while we are still active, and not on heading.
        while (_opMode.opModeIsActive() &&
                _runtime.seconds() < timeoutS &&
                !onHeading(speed, angle)) {
            // Update telemetry & Allow time for other processes to run.
            _opMode.idle();
        }

        // Stop all motion;
        _robot.setDrivePower(0d, 0d, 0d, 0d);
    }

    private boolean onHeading(double speed, double angle) {
        double error;
        double steer;
        boolean onTarget = false;
        double speedLeftFront, speedLeftRear, speedRightFront, speedRightRear;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0d;
            speedLeftFront = speedLeftRear = speedRightFront = speedRightRear = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, P_TURN_COEFF);
            speedRightFront = speedRightRear = speed * steer;
            speedLeftFront = speedLeftRear = speed * -steer;
        }

        // Send desired speeds to motors.
        _robot.setDriveVelocity(speedLeftFront,
                speedLeftRear,
                speedRightFront,
                speedRightRear);

        return onTarget;
    }

    private double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - _robot.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1d, 1d);
    }

    private double getSteer(double error, double PCoeff, double power) {
        return Range.scale(error * PCoeff, -1, 1d, -power, power);
    }
}
