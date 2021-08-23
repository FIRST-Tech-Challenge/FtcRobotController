package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Log;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Utils;

public class RearWheelDrive extends AbstractDriveTrain {
    private static final Log LOG = Logger.getInstance();

    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    public RearWheelDrive(DcMotor motorBackLeft, DcMotor motorBackRight) {
        this.motorBackLeft = motorBackLeft;
        this.motorBackRight = motorBackRight;
    }

    @Override
    public void init() {
        setDirection(motorBackLeft, DcMotor.Direction.REVERSE);

        setZeroPowerBehavior(motorBackLeft, DcMotor.ZeroPowerBehavior.BRAKE);
        setZeroPowerBehavior(motorBackRight, DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void reset() {
        LOG.log(Logger.CAPTION.Status, "Resetting Wheel Encoders");
        setMode(motorBackLeft, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorBackRight, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMode(motorBackLeft, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(motorBackRight, DcMotor.RunMode.RUN_USING_ENCODER);

        logPosition();
    }

    @Override
    public void driveByTime(double power, long time) {
        setPowerToAllMotors(power);

        Utils.sleep(time);

        setPowerToAllMotors(Constants.ZERO_POWER);
    }


    private void setPowerToAllMotors(double power) {
        setPower(motorBackLeft, power);
        setPower(motorBackRight, power);
    }

    private void setModeToAllMotors(DcMotor.RunMode runMode) {
        setMode(motorBackLeft, runMode);
        setMode(motorBackRight, runMode);
    }

    @Override
    public void driveStraight(double power, long time) {
        driveByTime(power, time);
    }

    @Override
    public void drive(double speed, double leftInches, double rightInches, double timeout) {
        int newBackLeftTarget;
        int newBackRightTarget;

        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (isOpModeActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int) (leftInches * Constants.COUNTS_PER_INCH);
            newBackRightTarget = motorBackRight.getCurrentPosition() + (int) (rightInches * Constants.COUNTS_PER_INCH);

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

    @Override
    public void turnRight(int degrees) {
        float angularOrientation = getImu().getAngularOrientation();
        LOG.log(Log.CAPTION.Position, angularOrientation);

        Utils.sleep(500);

        motorBackLeft.setPower(0.2);
        motorBackRight.setPower(-0.2);

        while (angularOrientation > degrees && !isStopRequested()) {
            angularOrientation = super.getImu().getAngularOrientation();
            LOG.log(Log.CAPTION.Position, angularOrientation);
        }
        setPowerToAllMotors(Constants.ZERO_POWER);

        Utils.sleep(250);
    }

    @Override
    public void turnLeft(int degrees) {
        float angularOrientation = getImu().getAngularOrientation();
        LOG.log(Log.CAPTION.Position, angularOrientation);

        Utils.sleep(500);

        motorBackLeft.setPower(-0.2);
        motorBackRight.setPower(0.2);

        while (angularOrientation < degrees && !isStopRequested()) {
            angularOrientation = super.getImu().getAngularOrientation();
            LOG.log(Log.CAPTION.Position, angularOrientation);
        }
        setPowerToAllMotors(Constants.ZERO_POWER);

        Utils.sleep(250);
    }

    public void logPosition() {
        LOG.log(Logger.CAPTION.Position, "Starting at %7d :%7d", motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition());
    }
}
