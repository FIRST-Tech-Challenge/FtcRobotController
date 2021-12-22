package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

import java.util.Arrays;

public abstract class MasterAutonomous extends MasterOpMode {
    public double max(double a, double b, double c) {
        double[] array = {a, b, c};
        Arrays.sort(array);

        return array[2];
    }

    // This method drives tank when given an angle drive power and turning power
    public void driveTank(double leftSidePower, double rightSidePower) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setPower(leftSidePower);
        motorBackLeft.setPower(leftSidePower);
        motorFrontRight.setPower(rightSidePower);
        motorBackRight.setPower(rightSidePower);
    }

    // This method drives a specified number of inches in a straight line when given a target distance and max speed
    // Set direction to false when going forward and true when going backwards
    public void driveInches(double targetDistance, double minSpeed, boolean backwards) {
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

        while (!distanceReached && opModeIsActive()) {
            // This calculates the angle deviation
            angleDeviation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle;

            if (backwards) {
                distanceLeft = targetDistance + position;
            } else {
                distanceLeft = targetDistance - position;
            }

            if (backwards) {
                driveTank(max(distanceLeft / 48, minSpeed, Constants.MIN_DRIVE_PWR) * -1,
                        max(distanceLeft / 48, minSpeed, Constants.MIN_DRIVE_PWR) * -1);
            } else {
                driveTank(max(distanceLeft / 48, minSpeed, Constants.MIN_DRIVE_PWR),
                        max(distanceLeft / 48, minSpeed, Constants.MIN_DRIVE_PWR));
            }

            // todo - find a way to regulate the angle without the robot stopping half way through (maybe a way to concurrently turn and drive?)
            /*if (Math.abs(angleDeviation) >= 1) {
                turnDegrees(angleDeviation * -1);
            }*/

            // Update positions using last distance measured by encoders
            position = Constants.IN_PER_AM_TICK * (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition() -
                    motorFrontRight.getCurrentPosition() - motorBackRight.getCurrentPosition()) / 4.0;

            if (backwards) {
                if (Math.abs(position + targetDistance) <= 0.5) {
                    driveTank(0.0, 0.0);
                    distanceReached = true;
                }
            } else {
                if (Math.abs(position - targetDistance) <= 0.5) {
                    driveTank(0.0, 0.0);
                    distanceReached = true;
                }
            }
        }
    }

    // This method turns a specified number of degrees when given a target angle to turn
    public void turnToAngle(double targetAngle) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angleLeft;
        boolean angleReached = false;

        while (!angleReached && opModeIsActive()) {
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // This gets the angle change
            angleLeft = targetAngle - currentAngle;

            if (Math.abs(angleLeft) > 180 && angleLeft > 180) {
                angleLeft = angleLeft - 360;
            } else if (Math.abs(angleLeft) > 180 && angleLeft < 180) {
                angleLeft = angleLeft + 360;
            }

            if (angleLeft > 0) {
                driveTank(Math.max(angleLeft / 90, Constants.MIN_TURN_PWR),
                        Math.max(angleLeft / 90, Constants.MIN_TURN_PWR) * -1);
            } else if (angleLeft < 0) {
                driveTank(Math.min(angleLeft / 90, Constants.MIN_TURN_PWR * -1),
                        Math.max(angleLeft / 90, Constants.MIN_TURN_PWR));
            }

            if (Math.abs(angleLeft) <= 0.5) {
                driveTank(0.0, 0.0);
                angleReached = true;
            }
        }
    }
}