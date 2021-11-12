package org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IMU {

    private final BNO055IMU imu;

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

    public double getAngle() {
        double returnVal;
        if (imu.getAngularOrientation().firstAngle < 0) {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle);
        } else {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle - 360);
        }
        return returnVal % 360;

    }

    public BNO055IMU getImu() {
        return imu;
    }
}
