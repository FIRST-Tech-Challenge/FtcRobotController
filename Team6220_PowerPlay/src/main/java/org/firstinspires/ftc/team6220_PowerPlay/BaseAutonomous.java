package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class BaseAutonomous extends BaseOpMode {
    DcMotorEx[] motors = {motorFL, motorFR, motorBL, motorBR};

    public void initialize() {
        super.initialize();

        /*
        for (DcMotorEx motor : motors) {
            motor.setVelocityPIDFCoefficients();
        }
        */
    }

    /**
     * this method will allow the robot to drive straight in a specified direction given a specified heading and distance
     * @param heading 360-degree direction robot should move (front is 0)
     * @param targetDistance distance robot should move in inches
     */
    public void driveInches(int heading, double targetDistance) {
        double powerPerInch = Constants.POWER_40 / Constants.ACCEL_DECEL_LENGTH;

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double turningPower;

        double distanceTraveled = 0.0;
        double distanceLeft = targetDistance;

        double motorPowerTraveled = 0.0;
        double motorPowerLeft = 0.0;

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (targetDistance > Constants.ACCEL_DECEL_LENGTH * 2) {
            while (distanceLeft > 0 && opModeIsActive()) {
                turningPower = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle) / 100;

                if (distanceTraveled < Constants.ACCEL_DECEL_LENGTH) {
                    motorFL.setPower(Math.max(motorPowerTraveled + turningPower, 0.1));
                    motorFR.setPower(Math.max(motorPowerTraveled - turningPower, 0.1));
                    motorBL.setPower(Math.max(motorPowerTraveled + turningPower, 0.1));
                    motorBR.setPower(Math.max(motorPowerTraveled - turningPower, 0.1));
                } else if (distanceLeft > Constants.ACCEL_DECEL_LENGTH) {
                    motorFL.setPower(Constants.POWER_40 + turningPower);
                    motorFR.setPower(Constants.POWER_40 - turningPower);
                    motorBL.setPower(Constants.POWER_40 + turningPower);
                    motorBR.setPower(Constants.POWER_40 - turningPower);
                } else {
                    motorFL.setPower(Math.max(motorPowerLeft + turningPower, 0.1));
                    motorFR.setPower(Math.max(motorPowerLeft - turningPower, 0.1));
                    motorBL.setPower(Math.max(motorPowerLeft + turningPower, 0.1));
                    motorBR.setPower(Math.max(motorPowerLeft - turningPower, 0.1));
                }

                distanceTraveled = getDistanceTraveled();
                distanceLeft = targetDistance - distanceTraveled;

                motorPowerTraveled = distanceTraveled * powerPerInch;
                motorPowerLeft = distanceLeft * powerPerInch;
            }
        } else {
            while (distanceLeft > 0 && opModeIsActive()) {
                turningPower = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle) / 100;

                if (distanceTraveled < targetDistance / 2) {
                    motorFL.setPower(Math.max(motorPowerTraveled + turningPower, 0.1));
                    motorFR.setPower(Math.max(motorPowerTraveled - turningPower, 0.1));
                    motorBL.setPower(Math.max(motorPowerTraveled + turningPower, 0.1));
                    motorBR.setPower(Math.max(motorPowerTraveled - turningPower, 0.1));
                } else {
                    motorFL.setPower(Math.max(motorPowerLeft + turningPower, 0.1));
                    motorFR.setPower(Math.max(motorPowerLeft - turningPower, 0.1));
                    motorBL.setPower(Math.max(motorPowerLeft + turningPower, 0.1));
                    motorBR.setPower(Math.max(motorPowerLeft - turningPower, 0.1));
                }

                distanceTraveled = getDistanceTraveled();
                distanceLeft = targetDistance - distanceTraveled;

                motorPowerTraveled = distanceTraveled * powerPerInch;
                motorPowerLeft = distanceLeft * powerPerInch;
            }
        }

        for (DcMotorEx motor : motors) {
            motor.setPower(0.0);
        }
    }

    /**
     * this method will allow the robot to drive straight in a specified direction using the IMU given a specified heading and distance
     * @param heading 360-degree direction robot should turn to (front is 0)
     */
    public void turnToAngle(int heading) {
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double currentAngle = startAngle;
        double error;

        if (heading > startAngle) {
            while (currentAngle < heading) {
                error = heading - currentAngle;

                motorFL.setPower(Math.min(-error * 0.02, -0.1));
                motorFR.setPower(Math.max(error * 0.02, 0.1));
                motorBL.setPower(Math.min(-error * 0.02, -0.1));
                motorBR.setPower(Math.max(error * 0.02, 0.1));

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        } else {
            while (heading < currentAngle) {
                error = currentAngle - heading;

                motorFL.setPower(Math.max(error * 0.02, 0.1));
                motorFR.setPower(Math.min(-error * 0.02, -0.1));
                motorBL.setPower(Math.max(error * 0.02, 0.1));
                motorBR.setPower(Math.min(-error * 0.02, -0.1));

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
        }

        for (DcMotorEx motor : motors) {
            motor.setPower(0.0);
        }
    }

    public double getDistanceTraveled() {
        double yPosition;
        double xPosition;
        double distanceTraveled;

        xPosition = (motorFL.getCurrentPosition() - motorFR.getCurrentPosition() - motorBL.getCurrentPosition() + motorBR.getCurrentPosition()) * Constants.DRIVE_MOTOR_TICKS_TO_INCHES / 4.0;
        yPosition = (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBL.getCurrentPosition() + motorBR.getCurrentPosition()) * Constants.DRIVE_MOTOR_TICKS_TO_INCHES / 4.0;

        distanceTraveled = Math.sqrt(Math.pow(xPosition, 2) + Math.pow(yPosition, 2));

        return distanceTraveled;
    }
}
