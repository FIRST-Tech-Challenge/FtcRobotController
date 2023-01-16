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
        if(imu.getRevIMU() instanceof BNO055IMU) {
            accel = imu.getXYZGs();

            maxAccelX = accel[0] > maxAccelX ? accel[0] : maxAccelX;
            maxAccelY = accel[1] > maxAccelY ? accel[1] : maxAccelY;
            maxAccelZ = accel[2] > maxAccelZ ? accel[2] : maxAccelZ;

            telemetry.addData("Accel X: ", accel[0]);
            telemetry.addData("Accel Y: ", accel[1]);
            telemetry.addData("Accel Z: ", accel[2]);
            telemetry.addData("Max Accel X: ", maxAccelX);
            telemetry.addData("Max Accel Y: ", maxAccelY);
            telemetry.addData("Max Accel Z: ", maxAccelZ);

            dashboardTelemetry.addData("Accel X: ", accel[0]);
            dashboardTelemetry.addData("Accel Y: ", accel[1]);
            dashboardTelemetry.addData("Accel Z: ", accel[2]);
            dashboardTelemetry.addData("Max Accel X: ", maxAccelX);
            dashboardTelemetry.addData("Max Accel Y: ", maxAccelY);
            dashboardTelemetry.addData("Max Accel Z: ", maxAccelZ);
        }

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
}