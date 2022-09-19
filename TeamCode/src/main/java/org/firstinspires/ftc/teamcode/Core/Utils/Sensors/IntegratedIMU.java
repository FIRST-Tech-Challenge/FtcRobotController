//
// Created by Neil Rodriguez 12/07/2021
//

package org.firstinspires.ftc.teamcode.Core.Utils.Sensors;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Core.BaseClasses.EctoGyroscope;
import org.firstinspires.ftc.teamcode.Core.BaseClasses.EctoIMU;

public class IntegratedIMU extends EctoIMU implements EctoGyroscope {

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub
     * @param hardwareMap      Hardware map
     */
    public IntegratedIMU( HardwareMap hardwareMap, String moduleName) {
        super(moduleName, "Sensor");
        imu = hardwareMap.get(BNO055IMU.class, moduleName);
        multiplier = 1;
    }

    private final BNO055IMU imu;

    /***
     * Heading relative to starting position
     */
    double globalHeading;

    /**
     * Heading relative to last offset
     */
    double relativeHeading;

    /**
     * Offset between global heading and relative heading
     */
    public double offset;

    private int multiplier;


    /**
     * Initializes gyro with custom parameters.
     */
    public void init(BNO055IMU.Parameters parameters) {
        imu.initialize(parameters);

        globalHeading = 0;
        relativeHeading = 0;
        offset = 0;
    }

    /**
     * Inverts the ouptut of gyro
     */
    public void invertGyro() {
        multiplier *= -1;
    }

    /**
     * @return Relative heading of the robot
     */
    public double getHeading() {
        globalHeading = imu.getAngularOrientation().firstAngle;
        relativeHeading = globalHeading + offset;
        // Return yaw
        return relativeHeading * multiplier;
    }

    /**
     * @return Absolute heading of the robot
     */
    public double getAbsoluteHeading() {
        return imu.getAngularOrientation().firstAngle * multiplier;
    }

    /**
     * @return X, Y, Z angles of gyro
     */
    public double[] getAngles() {
        // make a singular hardware call
        Orientation orientation = imu.getAngularOrientation();

        return new double[]{orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle};
    }

    /**
     * @return Transforms heading into {@link Rotation2d}
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }


    public void reset() {
        offset = -getHeading();
    }


    /**
     * Initializes gyro with default parameters.
     */
    @Override
    public void initSensor() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit =                              BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit =                              BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile =                    "BNO055IMUCalibration.json";
        parameters.loggingEnabled =                         true;
        parameters.loggingTag =                             "IMU";
        parameters.accelerationIntegrationAlgorithm = new   JustLoggingAccelerationIntegrator();

        init(parameters);
    }

    @Override
    public void startSensor() {

    }

    @Override
    public void updateSensor() {

    }

    @Override
    public void stopSensor() {

    }
}