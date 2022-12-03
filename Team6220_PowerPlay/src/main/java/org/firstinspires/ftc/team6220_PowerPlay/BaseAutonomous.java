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
     * this method will allow the robot to drive straight in a specified direction using the IMU given a specified heading and distance
     * @param degrees number of degrees robot should turn (positive is counterclockwise and negative is clockwise)
     */
    public void turnDegrees(double degrees) {
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double currentAngle;

        double remainingAngle = degrees;
        double traveledAngle;

        while (remainingAngle > 0 && opModeIsActive()) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // robot is turning counter-clockwise
            if (degrees > 0) {
                motorFL.setPower(Math.min(-remainingAngle / 180.0, -0.1));
                motorFR.setPower(Math.min(-remainingAngle / 180.0, -0.1));
                motorBL.setPower(Math.max(remainingAngle / 180.0, 0.1));
                motorBR.setPower(Math.max(remainingAngle / 180.0, 0.1));

            // robot is turning clockwise
            } else {
                motorFL.setPower(Math.max(-remainingAngle / 180.0, 0.1));
                motorFR.setPower(Math.max(-remainingAngle / 180.0, 0.1));
                motorBL.setPower(Math.min(remainingAngle / 180.0, -0.1));
                motorBR.setPower(Math.min(remainingAngle / 180.0, -0.1));
            }

            traveledAngle = currentAngle - startAngle;
            remainingAngle = degrees - traveledAngle;
        }

        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }
}
