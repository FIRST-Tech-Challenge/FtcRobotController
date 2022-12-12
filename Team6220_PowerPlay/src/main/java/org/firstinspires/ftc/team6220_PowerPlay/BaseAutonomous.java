package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class BaseAutonomous extends BaseOpMode {

    /**
     * this method will allow the robot to drive straight in a specified direction given a specified heading and distance
     * @param heading 360-degree direction robot should move (front is 0)
     * @param targetDistance distance robot should move in inches
     */
    public void driveInches(double heading, double targetDistance) {
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double turningPower;

        // looking at robot from the back, x is left/right and y is forwards/backwards
        double xPosition;
        double yPosition;

        double traveledDistance;
        double remainingDistance = targetDistance;

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (remainingDistance > 0 && opModeIsActive()) {
            turningPower = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle) / 100.0;

            motorFL.setPower(0.3 + turningPower);
            motorFR.setPower(0.3 - turningPower);
            motorBL.setPower(0.3 + turningPower);
            motorBR.setPower(0.3 - turningPower);

            xPosition = (motorFL.getCurrentPosition() - motorFR.getCurrentPosition() - motorBL.getCurrentPosition() + motorBR.getCurrentPosition()) * Constants.DRIVE_MOTOR_TICKS_TO_INCHES / 4.0;
            yPosition = (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBL.getCurrentPosition() + motorBR.getCurrentPosition()) * Constants.DRIVE_MOTOR_TICKS_TO_INCHES / 4.0;

            traveledDistance = Math.sqrt(Math.pow(xPosition, 2) + Math.pow(yPosition, 2));
            remainingDistance = targetDistance - traveledDistance;
        }

        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    /**
     * this method will allow the robot to turn to a specified absolute angle using the IMU
     * @param targetAngle absolute angle robot should turn to
     */
    public void turnToAngle(double targetAngle) {
        double currentAngle;
        double angleError = 1.0;
        double motorPower;

        while (Math.abs(angleError) >= 1 && opModeIsActive()) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            angleError = targetAngle - currentAngle + 45.0;

            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            // robot is turning counter-clockwise
            if (angleError > 0) {
                motorPower = Math.min(angleError / -180.0, -0.2);

            // robot is turning clockwise
            } else {
                motorPower = Math.max(angleError / -180.0, 0.2);
            }

            motorFL.setPower(motorPower);
            motorFR.setPower(-motorPower);
            motorBL.setPower(motorPower);
            motorBR.setPower(-motorPower);

            telemetry.addData("current", currentAngle);
            telemetry.addData("error", angleError);
            telemetry.addData("power", motorPower);
            telemetry.update();
        }

        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }
}
