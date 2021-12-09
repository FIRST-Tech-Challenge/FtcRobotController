package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.PIDFilter;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

public abstract class MasterAutonomous extends MasterOpMode {
    // This method drives tank when given an angle drive power and turning power
    public void driveTank(double leftSidePower, double rightSidePower) {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setPower(leftSidePower);
        motorBackLeft.setPower(leftSidePower);
        motorFrontRight.setPower(rightSidePower);
        motorBackRight.setPower(rightSidePower);

//        pauseMillis(time);
//
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//        motorFrontRight.setPower(0);
//        motorBackRight.setPower(0);
    }

    // This method drives a specified number of inches in a straight line when given a target distance and max speed
    // Set direction to true when going forward and false when going backwards
    // todo - test this method
    public void driveInches(double targetDistance, double maxSpeed, boolean direction) {
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
                driveTank(Math.min(Math.max(translationPID.getFilteredValue(), Constants.MINIMUM_DRIVE_POWER), maxSpeed),
                        Math.min(Math.max(translationPID.getFilteredValue(), Constants.MINIMUM_DRIVE_POWER), maxSpeed));
            } else {
                driveTank(Math.min(Math.max(translationPID.getFilteredValue() * -1, Constants.MINIMUM_DRIVE_POWER), maxSpeed) * -1,
                        Math.min(Math.max(translationPID.getFilteredValue() * -1, Constants.MINIMUM_DRIVE_POWER), maxSpeed) * -1);
            }

//            if (Math.abs(angleDeviation) >= 1) {
//                turnDegrees(angleDeviation * -1);
//            }

            // Update positions using last distance measured by encoders
            position = Constants.IN_PER_AM_TICK * (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition() +
                    motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition()) / 4.0;

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
    // todo - test this method
    public void turnDegrees(double targetAngle) {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
                driveTank(Math.max((translationPID.getFilteredValue() / 5), Constants.MINIMUM_TURNING_POWER),
                        Math.min((translationPID.getFilteredValue() / 5), Constants.MINIMUM_TURNING_POWER * -1));
            } else if (targetAngle < 0) {
                driveTank(Math.min((translationPID.getFilteredValue() / 5), Constants.MINIMUM_TURNING_POWER * -1),
                        Math.max((translationPID.getFilteredValue() / 5), Constants.MINIMUM_TURNING_POWER));
            }

            if (Math.abs(targetAngle - angleTraveled) <= 1) {
                driveTank(0.0, 0.0);
                angleReached = true;
            }
        }
    }
}