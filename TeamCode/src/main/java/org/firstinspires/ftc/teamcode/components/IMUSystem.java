package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Creates an IMU system that handles the angle and movement of the robot through a gyroscope.
 */
public class IMUSystem {
    public BNO055IMU imu;
    public BNO055IMU.Parameters parameters;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    /**
     * Creates a new IMU System
     */
    public IMUSystem(BNO055IMU imu)
    {
        this.parameters                = new BNO055IMU.Parameters();
        this.parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        this.parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.parameters.loggingEnabled = true;
        this.parameters.loggingTag     = "BNO055";
        // this.parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        this.imu = imu;
        this.imu.initialize(parameters);

        // Enable reporting of position using the naive integrator
        imu.startAccelerationIntegration(new Position(), new Velocity(), 500);
    }

    /**
     * Gets the yaw of the IMU
     * @return Returns the yaw in degrees
     */
    public double getHeading() {
        return -imu.getAngularOrientation().firstAngle;
    }

    /**
     * Gets the roll of the IMU
     * @return Returns the roll in degrees
     */
    public double getRoll() {
        Orientation orientation = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return orientation.secondAngle;
    }

    /**
     * Gets the pitch of the IMU
     * @return Returns the pitch in degrees
     */
    public double getPitch() {
        Orientation orientation = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return orientation.thirdAngle;
    }

    /**
     * Gets the acceleration of the IMU
     * @return Returns the linear acceleration object
     * @see Acceleration
     */
    public Acceleration getAcceleration()
    {
        return imu.getLinearAcceleration();
    }

    /**
     * Gets the velocity of the IMU
     * @return Returns the velocity of the IMU
     * @see Velocity
     */
    public Velocity getVelocity()
    {
        return imu.getVelocity();
    }

    /**
     * Formats the angle as a string
     * @param angleUnit unit of the angle
     * @param angle value of the angle
     * @return Returns the formatted value of the angle
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    /**
     * Formats the degrees of the IMU
     * @param degrees default type of the degrees
     * @return Returns the formatted value of the degrees
     */
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}