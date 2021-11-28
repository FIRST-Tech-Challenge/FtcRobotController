package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.PIDFilter;


public abstract class MasterAutonomous extends MasterOpMode {
    // This method drives tank when given an angle drive power and turning power
    public void driveTank(double leftSidePower, double rightSidePower) {
        motorFrontLeft.setPower(leftSidePower);
        motorBackLeft.setPower(leftSidePower);
        motorFrontRight.setPower(rightSidePower);
        motorBackRight.setPower(rightSidePower);
    }


    public void turnDegrees(double targetAngle) {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
                driveTank(Math.max((translationPID.getFilteredValue() / 2) * -1, Constants.MINIMUM_TURNING_POWER),
                        Math.max((translationPID.getFilteredValue() / 5), Constants.MINIMUM_TURNING_POWER));
            } else {
                driveTank(Math.max((translationPID.getFilteredValue() / 5), Constants.MINIMUM_TURNING_POWER),
                        Math.max((translationPID.getFilteredValue() / 5) * -1, Constants.MINIMUM_TURNING_POWER));
            }

            if (Math.abs(targetAngle - angleTraveled) < 1) {
                driveTank(0.0, 0.0);
                angleReached = true;
            }
        }
    }
}