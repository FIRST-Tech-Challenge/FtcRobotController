package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class IMU {
    private BNO055IMU imu;
    private Telemetry telemetry;
    private Telemetry dashboardTelemetry;

    private double previousRawValue = 0;
    private double turns = 0;
    private double rawValue;
    private double contValue;

    public IMU(HardwareMap hardwareMap, Telemetry telemetry, Telemetry dashboardTelemetry) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        this.telemetry = telemetry;
        this.dashboardTelemetry = dashboardTelemetry;
    }

    public void update() {
        rawValue = -imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

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