package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class BaseAutonomous extends BaseOpMode {

    // this method will allow the robot to drive straight in a specified direction using the IMU given a specified power, heading, and distance
    public void driveOmniInches(double power, int headingDegrees, double targetDistance) {
        boolean distanceReached = false;

        double xPosition = 0;
        double yPosition = 0;
        double distanceLeft, distanceTraveled, angleDeviation, motorPower, turningPower;

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double headingRadians = Math.toRadians(headingDegrees);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!distanceReached) {
            angleDeviation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle;

            distanceTraveled = Math.sqrt(Math.pow((xPosition - 0), 2) + Math.pow((yPosition - 0), 2));

            turningPower = angleDeviation / 180;

            motorPower = distanceTraveled / targetDistance * Math.PI;

            if (headingRadians == 0) {
                motorFL.setPower(motorPower + turningPower);
                motorFR.setPower(motorPower - turningPower);
                motorBL.setPower(motorPower + turningPower);
                motorBR.setPower(motorPower - turningPower);
            } else if (headingRadians == Math.PI / 2) {
                motorFL.setPower(-motorPower + turningPower);
                motorFR.setPower(-motorPower - turningPower);
                motorBL.setPower(-motorPower + turningPower);
                motorBR.setPower(-motorPower - turningPower);
            } else if (headingRadians == Math.PI) {
                motorFL.setPower(-motorPower + turningPower);
                motorFR.setPower(motorPower - turningPower);
                motorBL.setPower(motorPower + turningPower);
                motorBR.setPower(-motorPower - turningPower);
            } else if (headingRadians == 3 * Math.PI / 2) {
                motorFL.setPower(motorPower + turningPower);
                motorFR.setPower(-motorPower - turningPower);
                motorBL.setPower(-motorPower + turningPower);
                motorBR.setPower(motorPower - turningPower);
            }

            xPosition = (Constants.CHASSIS_INCHES_PER_TICK * (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBL.getCurrentPosition() + motorBR.getCurrentPosition()) / 4);
            yPosition = (Constants.CHASSIS_INCHES_PER_TICK * (motorFL.getCurrentPosition() - motorFR.getCurrentPosition() - motorBL.getCurrentPosition() + motorBR.getCurrentPosition()) / 4);

            if (distanceTraveled > targetDistance) {
                motorFL.setPower(0.0);
                motorFR.setPower(0.0);
                motorBL.setPower(0.0);
                motorBR.setPower(0.0);

                distanceReached = true;
            }
        }
    }
}
