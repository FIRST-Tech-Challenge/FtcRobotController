package org.firstinspires.ftc.teamcode.Utils.Chassis;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.KodiakMath;

public class FieldOrientedDriver {
    public static PIDController headingPidController = new PIDController(2, 0, 0);
    public static double MAX_HEADING_PID_VALUE = 1.5;
    public static double HEADING_TOLERANCE = 0.05;

    private double targetAngle = 0;
    private static double calibratedForwardHeading = 0;
    private boolean needPidControlRotation = false;

    private IMU imu;

    public FieldOrientedDriver(IMU imu) {
        this.imu = imu;
    }

    public ChassisSpeeds getChassisSpeeds(double forwardValue, double leftValue, double rotationValue) {
        double rotation = rotationValue;

        double currentHeading = getCalibratedHeading();
        double currentError = 0;
        if (rotation == 0) {
            currentError = currentHeading - targetAngle;

            if (currentError > Math.PI) {
                currentError -= 2 * Math.PI;
            } else if (currentError < -Math.PI) {
                currentError += 2 * Math.PI;
            }

            if (!needPidControlRotation) {
                targetAngle = currentHeading;
                needPidControlRotation = true;
            }

            if (Math.abs(currentError) > HEADING_TOLERANCE) {
                rotation = KodiakMath.between(
                        headingPidController.calculate(currentError),
                        -MAX_HEADING_PID_VALUE,
                        MAX_HEADING_PID_VALUE
                );
            }
        } else {
            needPidControlRotation = false;
        }

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardValue,
                leftValue,
                rotation,
                new Rotation2d(currentHeading)
        );

        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond * ChassisDriver.STRAFE_POWER_MULTIPLIER, speeds.omegaRadiansPerSecond);

        return speeds;
    }

    public void setTargetHeadingRad(double a) {
        targetAngle = a;
    }

    public void resetCalibratedForwardHeading() {
        calibratedForwardHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        targetAngle = 0;
    }

    public void setCalibratedForwardHeading(double a) {
        calibratedForwardHeading = a;
    }

    public double getCalibratedHeading() {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - calibratedForwardHeading;

        if (currentHeading > Math.PI) {
            currentHeading -= 2 * Math.PI;
        } else if (currentHeading < -Math.PI) {
            currentHeading += 2 * Math.PI;
        }

        return currentHeading;
    }
}
