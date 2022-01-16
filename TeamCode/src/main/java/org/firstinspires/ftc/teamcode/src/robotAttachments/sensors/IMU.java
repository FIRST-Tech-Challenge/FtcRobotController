package org.firstinspires.ftc.teamcode.src.robotAttachments.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A wrapper class for the BNO055IMU
 */
public class IMU {

    /**
     * The internal IMU object
     */
    private final BNO055IMU imu;

    /**
     * Initializes Imu off of hardware map and name
     *
     * @param hardwareMap OpMode hardware map
     * @param deviceName  Name of IMU sensor
     */
    public IMU(HardwareMap hardwareMap, String deviceName) {

        imu = hardwareMap.get(BNO055IMU.class, deviceName);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

    }

    /**
     * Returns the IMU angle on the plane parallel to the floor
     *
     * @return Returns the angle in degrees
     */
    public double getAngle() {
        double returnVal;
        if (imu.getAngularOrientation().firstAngle < 0) {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle);
        } else {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle - 360);
        }
        return returnVal % 360;

    }

    /**
     * Returns the internal IMU object
     *
     * @return returns the Internal IMU object
     */
    public BNO055IMU getImu() {
        return imu;
    }
}
