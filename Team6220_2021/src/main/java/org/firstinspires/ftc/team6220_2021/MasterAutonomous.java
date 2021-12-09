package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.PIDFilter;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

public abstract class MasterAutonomous extends MasterOpMode {
    public double max(double a, double b, double c) {
        if (a > b && a > c) {
            return a;
        } else if (b > a && b > c) {
            return b;
        } else if (c > b && c > a) {
            return c;
        } else {
            return a;
        }
    }

    // This method drives tank when given an angle drive power and turning power
    public void driveTank(double leftSidePower, double rightSidePower) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setPower(leftSidePower);
        motorBackLeft.setPower(leftSidePower);
        motorFrontRight.setPower(-rightSidePower);
        motorBackRight.setPower(-rightSidePower);
    }

    // This method drives a specified number of inches in a straight line when given a target distance and max speed
    // Set direction to true when going forward and false when going backwards
    public void driveInches(double targetDistance, double minSpeed, boolean direction) {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean distanceReached = false;

        double position = 0.0;
        double distanceLeft;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angleDeviation;

        PIDFilter translationPID;
        translationPID = new PIDFilter(Constants.TRANSLATION_P, Constants.TRANSLATION_I, Constants.TRANSLATION_D);

        while (!distanceReached && opModeIsActive()) {
            // This calculates the angle deviation
            angleDeviation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle;

            // This adds a value to the PID loop so it can update
            distanceLeft = targetDistance - position;
            translationPID.roll(distanceLeft);

            // We drive the wheels with the PID value
            if (direction) {
                driveTank(max(translationPID.getFilteredValue(), minSpeed, Constants.MINIMUM_DRIVE_POWER),
                        max(translationPID.getFilteredValue(), minSpeed, Constants.MINIMUM_DRIVE_POWER));
            } else {
                driveTank(max(translationPID.getFilteredValue() * -1, minSpeed, Constants.MINIMUM_DRIVE_POWER) * -1,
                        max(translationPID.getFilteredValue() * -1, minSpeed, Constants.MINIMUM_DRIVE_POWER) * -1);
            }

            // todo - find a way to regulate the angle without the robot stopping half way through (maybe a way to concurrently turn and drive?)
            /*if (Math.abs(angleDeviation) >= 1) {
                turnDegrees(angleDeviation * -1);
            }*/

            // Update positions using last distance measured by encoders
            position = Constants.IN_PER_AM_TICK * (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition() -
                    motorFrontRight.getCurrentPosition() - motorBackRight.getCurrentPosition()) / 4.0;

            if (direction) {
                if (Math.abs(position - targetDistance) <= 1) {
                    driveTank(0.0, 0.0);
                    distanceReached = true;
                }
            } else {
                if (Math.abs(position + targetDistance) <= 1) {
                    driveTank(0.0, 0.0);
                    distanceReached = true;
                }
            }
        }
    }

    // This method turns a specified number of degrees when given a target angle to turn
    public void turnDegrees(double targetAngle) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angleLeft;
        double angleTraveled;

        boolean angleReached = false;

        PIDFilter translationPID;
        translationPID = new PIDFilter(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);

        while (!angleReached && opModeIsActive()) {
            // This gets the angle change
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            angleTraveled = currentAngle - startAngle;

            // This adds a value to the PID loop so it can update
            angleLeft = targetAngle - angleTraveled;
            translationPID.roll(angleLeft);

            // We drive the wheels with the PID value
            if (targetAngle > 0) {
                driveTank(Math.max(translationPID.getFilteredValue(), Constants.MINIMUM_TURNING_POWER),
                        Math.max(translationPID.getFilteredValue(), Constants.MINIMUM_TURNING_POWER) * -1);
            } else if (targetAngle < 0) {
                driveTank(Math.max(translationPID.getFilteredValue(), Constants.MINIMUM_TURNING_POWER) * -1,
                        Math.max(translationPID.getFilteredValue(), Constants.MINIMUM_TURNING_POWER));
            }

            if (Math.abs(targetAngle - angleTraveled) <= 1) {
                driveTank(0.0, 0.0);
                angleReached = true;
            }
        }
    }
}