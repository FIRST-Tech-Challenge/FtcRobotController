package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.util.Constants.AXLE_DISTANCE_IN_INCHES;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Log;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Utils;

public class FourWheelDrive extends AbstractDriveTrain {
    private static final Log LOG = Logger.getInstance();

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    public FourWheelDrive(DcMotor motorFrontLeft, DcMotor motorFrontRight, DcMotor motorBackLeft, DcMotor motorBackRight) {
        this.motorFrontLeft = motorFrontLeft;
        this.motorFrontRight = motorFrontRight;
        this.motorBackLeft = motorBackLeft;
        this.motorBackRight = motorBackRight;
    }

    public void init() {
        setDirection(motorFrontLeft, DcMotor.Direction.REVERSE);
        setDirection(motorBackLeft, DcMotor.Direction.REVERSE);

        setZeroPowerBehavior(motorFrontLeft, DcMotor.ZeroPowerBehavior.BRAKE);
        setZeroPowerBehavior(motorFrontRight, DcMotor.ZeroPowerBehavior.BRAKE);
        setZeroPowerBehavior(motorBackLeft, DcMotor.ZeroPowerBehavior.BRAKE);
        setZeroPowerBehavior(motorBackRight, DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        LOG.log(Logger.CAPTION.Status, "Resetting Wheel Encoders");

        setMode(motorFrontLeft, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorFrontRight, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorBackLeft, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorBackRight, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMode(motorFrontLeft, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(motorFrontRight, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(motorBackLeft, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(motorBackRight, DcMotor.RunMode.RUN_USING_ENCODER);

        logPosition();
    }

    public void driveByTime(double power, long time) {
        setPowerToAllMotors(power);

        Utils.sleep(time);

        setPowerToAllMotors(Constants.ZERO_POWER);
    }

    private void setPowerToAllMotors(double power) {
        setPower(motorFrontLeft, power);
        setPower(motorFrontRight, power);
        setPower(motorBackLeft, power);
        setPower(motorBackRight, power);
    }

    private void setModeToAllMotors(DcMotor.RunMode runMode) {
        setMode(motorFrontLeft, runMode);
        setMode(motorFrontRight, runMode);
        setMode(motorBackLeft, runMode);
        setMode(motorBackRight, runMode);
    }

    public void driveStraight(double power, long time) {
        driveByTime(power, time);
    }

    public void turnRight(int degrees) {
        float angularOrientation = getImu().getAngularOrientation();
        LOG.log(Log.CAPTION.Position, angularOrientation);

        Utils.sleep(500);

        motorFrontLeft.setPower(0.2);
        motorBackLeft.setPower(0.2);

        motorFrontRight.setPower(-0.2);
        motorBackRight.setPower(-0.2);

        while (angularOrientation > degrees && !isStopRequested()) {
            angularOrientation = super.getImu().getAngularOrientation();
            LOG.log(Log.CAPTION.Position, angularOrientation);
        }
        setPowerToAllMotors(Constants.ZERO_POWER);

        Utils.sleep(250);
    }

    public void turnLeft(int degrees) {
        float angularOrientation = getImu().getAngularOrientation();
        LOG.log(Log.CAPTION.Position, angularOrientation);

        Utils.sleep(500);

        motorFrontLeft.setPower(-0.2);
        motorBackLeft.setPower(-0.2);

        motorFrontRight.setPower(0.2);
        motorBackRight.setPower(0.2);

        while (angularOrientation < degrees && !isStopRequested()) {
            angularOrientation = super.getImu().getAngularOrientation();
            LOG.log(Log.CAPTION.Position, angularOrientation);
        }
        setPowerToAllMotors(Constants.ZERO_POWER);

        Utils.sleep(250);
    }

    public void drive(double speed, double leftInches, double rightInches, double timeout) {
        int newBackLeftTarget;
        int newBackRightTarget;

        int newFrontLeftTarget;
        int newFrontRightTarget;

        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (isOpModeActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int) (leftInches * Constants.COUNTS_PER_INCH);
            newBackRightTarget = motorBackRight.getCurrentPosition() + (int) (rightInches * Constants.COUNTS_PER_INCH);

            //Decrease leftInches and rightInches by the length of the robot for the back motors
            newBackLeftTarget = newBackLeftTarget - AXLE_DISTANCE_IN_INCHES;
            newBackRightTarget = newBackRightTarget - AXLE_DISTANCE_IN_INCHES;

            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + (int) (leftInches * Constants.COUNTS_PER_INCH);
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + (int) (rightInches * Constants.COUNTS_PER_INCH);

            setTargetPosition(motorFrontLeft, newFrontLeftTarget);
            setTargetPosition(motorFrontRight, newFrontRightTarget);
            setTargetPosition(motorBackLeft, newBackLeftTarget);
            setTargetPosition(motorBackRight, newBackRightTarget);

            // Turn On RUN_TO_POSITION
            setModeToAllMotors(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setPowerToAllMotors(speed);
            LOG.log(Logger.CAPTION.Position, "Running to %7d :%7d", newBackLeftTarget, newBackRightTarget);

            while (isOpModeActive()
                    && (runtime.seconds() < timeout)
                    && (motorBackLeft.isBusy() && motorBackRight.isBusy())) {

                // Display it for the driver.
                LOG.log(Logger.CAPTION.Position, "Running to %7d :%7d", motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition());
            }

            // Stop all motion;
            setPowerToAllMotors(Constants.ZERO_POWER);

            // Turn off RUN_TO_POSITION
            setModeToAllMotors(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void logPosition() {
        LOG.log(Logger.CAPTION.Position, "Starting at %7d :%7d", motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition());
    }
}
