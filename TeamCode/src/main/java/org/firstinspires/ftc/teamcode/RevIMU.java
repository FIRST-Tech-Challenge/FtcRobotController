package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
//import com.qualcomm.hardware.IMU;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
public class RevIMU implements HardwareDevice {

    private IMU revIMU;

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
    double offset;

    private int multiplier;

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub
     *
     * @param hw      Hardware map
     * @param imuName Name of sensor in configuration
     */
    public RevIMU(HardwareMap hw, String imuName) {
        revIMU = hw.get(IMU.class, imuName);
        multiplier = 1;
    }

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub with the default configuration name of "imu"
     *
     * @param hw Hardware map
     */
    public RevIMU(HardwareMap hw) {
        this(hw, "imu");
    }

    /**
     * Initializes gyro with default parameters.
     */
    public void init() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

//        parameters.angleUnit = IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        init(parameters);
    }

    /**
     * Initializes gyro with custom parameters.
     */
    public void init(IMU.Parameters parameters) {
        revIMU.initialize(parameters);

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
     * @return Absolute heading of the robot
     */
//    @Override
//    public double getYaw() {
//        return revIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) * multiplier;
//    }
//
//    @Override
//    public double getPitch() {
//        return revIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) * multiplier;
//    }
//
//    @Override
//    public double getRoll() {
//        return revIMU.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) * multiplier;
//    }

    /**
     * @return X, Y, Z angles of gyro
     */
    public double[] getYawPitchRoll() {
        // make a singular hardware call
        YawPitchRollAngles ypr_angles = revIMU.getRobotYawPitchRollAngles();
        return new double[]{ypr_angles.getYaw(AngleUnit.DEGREES),
                ypr_angles.getPitch(AngleUnit.DEGREES), ypr_angles.getRoll(AngleUnit.DEGREES)};
    }

    public void disable() {
        revIMU.close();
    }

    /**
     * @return the internal sensor being wrapped
     */
    public IMU getRevIMU() {
        return revIMU;
    }

    @Override
    public String getDeviceType() {
        return "Rev Control Hub IMU";
    }

}
