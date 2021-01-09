package org.firstinspires.ftc.teamcode.breakout;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This class is used to simplify the usage of Gyroscopes in the code.
 * Please use this class instead of the BNO055IMU class.
 **/
public class BreakoutREVGyro {

    //Define objects
    Class<BNO055IMU> IMU = BNO055IMU.class;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    //State for telemetry
    Orientation angles;
    Acceleration gravity;

    public BNO055IMU get() {
        return imu;
    }

    public void set(BNO055IMU gyro) {
        this.imu = gyro;
    }

    public void calibrate() {
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    public boolean isCalibrating() {
        return !imu.isGyroCalibrated();
    }

    Acceleration getAccel() {
        return imu.getAcceleration();
    }

    Orientation getOrient(AxesReference reference, AxesOrder order, AngleUnit unit) {
        return imu.getAngularOrientation(reference, order, unit);
    }

    Velocity getVel() {
        return imu.getVelocity();
    }

    Position getPos() {
        return imu.getPosition();
    }

}
