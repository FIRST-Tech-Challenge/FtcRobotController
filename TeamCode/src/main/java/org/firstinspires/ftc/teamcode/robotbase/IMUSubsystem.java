package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.RevOldIMU;

public class IMUSubsystem extends SubsystemBase {
    private final RevIMU imu;
    private final Telemetry telemetry;
    private final Telemetry dashboardTelemetry;

    private double previousRawYaw = 0;
    private double turns = 0;
    private double rawYaw, rawPitch, rawRoll;
    private double[] accel;
    private double maxAccelX, maxAccelY, maxAccelZ;
    private double contYaw;

    private double[] angles;

    public IMUSubsystem(HardwareMap hardwareMap, Telemetry telemetry,
                        Telemetry dashboardTelemetry) {
        imu = new RevIMU(hardwareMap);
        imu.init();
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
//        That is inside init!!

    public void periodic() {
        angles = imu.getYawPitchRoll();
        rawYaw = angles[0];
        rawPitch = angles[1];
        rawRoll = angles[2];

        calculateContinuousValue();

        telemetry.addData("Gyro Yaw:", rawYaw);
        telemetry.addData("Gyro Pitch:", rawPitch);
        telemetry.addData("Gyro Roll:", rawRoll);
        telemetry.addData("Continuous Gyro Value:", contYaw);

        dashboardTelemetry.addData("Raw Gyro Value: ", rawYaw);
        dashboardTelemetry.addData("Gyro Pitch:", rawPitch);
        dashboardTelemetry.addData("Gyro Roll:", rawRoll);
        dashboardTelemetry.addData("Continuous Gyro Value:", contYaw);
    }

    public double getYaw() {
        return contYaw;
    }

    public double getRawYaw() {
        return rawYaw;
    }

    public double getPitch() {
        return rawPitch;
    }

    public double getRoll() {
        return rawRoll;
    }

    public int findClosestOrientationTarget() {
        double dist, minDist = Math.abs(contYaw);
        int minDistIdx = 0;
        int maxIdx = (int) Math.ceil(Math.abs(contYaw) / 90);
        for (int i = -maxIdx; i <= maxIdx; ++i) {
            dist = Math.abs(i * 90 - contYaw);
            if (dist < minDist) {
                minDistIdx = i;
                minDist = dist;
            }
        }

        return minDistIdx * 90;
    }

    private void calculateContinuousValue() {
        if (Math.abs(rawYaw - previousRawYaw) >= 180)
            turns += (rawYaw > previousRawYaw) ? -1 : 1;

        previousRawYaw = rawYaw;
        contYaw = rawYaw + 360 * turns;
    }

    public void resetYawValue() {
        imu.resetYaw();
    }
}