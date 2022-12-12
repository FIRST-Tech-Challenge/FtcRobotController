package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class IMUSubsystem extends SubsystemBase {
    private final RevIMU imu;
    private final Telemetry telemetry;
    private final Telemetry dashboardTelemetry;

    private double previousRawValue = 0;
    private double turns = 0;
    private double rawValue;
    private double contValue;

    public IMUSubsystem(HardwareMap hardwareMap, Telemetry telemetry,
                        Telemetry dashboardTelemetry) {
        imu = new RevIMU(hardwareMap);
        imu.init();
//        That is inside init!!
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        this.telemetry = telemetry;
        this.dashboardTelemetry = dashboardTelemetry;
    }

    public void periodic() {
        rawValue = imu.getHeading();

        calculateContinuousValue();

        telemetry.addData("Gyro Value:", rawValue);
        telemetry.addData("Continuous Gyro Value:", contValue);
        dashboardTelemetry.addData("Raw Gyro Value: ", rawValue);
    }

    public double getValue() {
        return contValue;
    }

    public double getRawValue() {
        return rawValue;
    }

    public int findClosestOrientationTarget() {
        double dist, minDist = Math.abs(contValue);
        int minDistIdx = 0;
        int maxIdx = (int) Math.ceil(Math.abs(contValue) / 90);
        for (int i = -maxIdx; i <= maxIdx; ++i) {
            dist = Math.abs(i * 90 - contValue);
            if (dist < minDist) {
                minDistIdx = i;
                minDist = dist;
            }
        }

        return minDistIdx * 90;
    }

    private void calculateContinuousValue() {
        if (Math.abs(rawValue - previousRawValue) >= 180)
            turns += (rawValue > previousRawValue) ? -1 : 1;

        previousRawValue = rawValue;
        contValue = rawValue + 360 * turns;
    }
}