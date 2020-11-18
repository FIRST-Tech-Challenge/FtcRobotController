package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Gyro extends SubSystem {

    BNO055IMU imu;
    Orientation angles;
    double heading = 0;
    double prevHeading = 0;

    public Gyro(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {

    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }

    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = robot.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public boolean isCalibrated() {
        return imu.isGyroCalibrated();
    }

    public String getCallibrationStatus() {
        return imu.getCalibrationStatus().toString();
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = convert(angles.firstAngle - prevHeading);
        robot.telemetry.addData("Gyro", "Heading - " + heading);
        robot.telemetry.update();
        return heading;
    }
    public void resetHeading() {
        prevHeading = heading;
    }

    public double devert (double degrees) {
        if (degrees < 0) {
            degrees += 360;
        }
        return degrees;
    }

    public double convert (double degrees) {
        if (degrees > 179) {
            degrees = -(360-degrees);
        }
        else if (degrees < -180) {
            degrees = 360 + degrees;
        }
        else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }
}
