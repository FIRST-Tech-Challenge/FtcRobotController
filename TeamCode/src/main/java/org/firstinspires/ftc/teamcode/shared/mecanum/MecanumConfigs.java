package org.firstinspires.ftc.teamcode.shared.mecanum;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

/**
 * This class allows the programmers to hold all physical drivetrain constants in one file. It also allows for quick adjustment if future robots
 * have different drivetrains.
 */
public class MecanumConfigs {
    // Default values. All values were tuned with the 2023-2024 robot in mind. Future iterations may require adjustments
    private Translation2d m_frontLeftPositionMeters = new Translation2d(0.178, 0.168);
    private Translation2d m_frontRightPositionMeters = new Translation2d(0.178, -0.168);
    private Translation2d m_backLeftPositionMeters = new Translation2d(-0.178, 0.168);
    private Translation2d m_backRightPositionMeters = new Translation2d(-0.178, -0.168);
    private double m_maxRobotSpeedMps = 3;
    private double m_maxRobotRotationRps = 6.28;
    private double m_metersPerTick = 0.00056; // Based on wheel geometry
    private Motor.RunMode m_runMode = Motor.RunMode.RawPower;
    private String m_frontLeftName = "fL";
    private String m_frontRightName = "fR";
    private String m_backLeftName = "bL";
    private String m_backRightName = "bR";

    public MecanumConfigs() {
    }

    public MecanumConfigs maxRobotSpeedMps(double maxRobotSpeedMps) {
        m_maxRobotSpeedMps = maxRobotSpeedMps;
        return this;
    }

    public MecanumConfigs maxRobotRotationRps(double maxRobotRotationRps) {
        m_maxRobotRotationRps = maxRobotRotationRps;
        return this;
    }

    public MecanumConfigs metersPerTick(double mpt) {
        m_metersPerTick = mpt;
        return this;
    }

    public MecanumConfigs runMode(Motor.RunMode mode) {
        m_runMode = mode;
        return this;
    }

    public double getMaxRobotSpeedMps() {
        return m_maxRobotSpeedMps;
    }

    public double getMaxRobotRotationRps() {
        return m_maxRobotRotationRps;
    }

    public String getFrontLeftName() {
        return m_frontLeftName;
    }

    public String getFrontRightName() {
        return m_frontRightName;
    }

    public String getBackLeftName() {
        return m_backLeftName;
    }

    public String getBackRightName() {
        return m_backRightName;
    }

    public Translation2d getFrontLeftPosition() {
        return m_frontLeftPositionMeters;
    }

    public Translation2d getFrontRightPosition() {
        return m_frontRightPositionMeters;
    }

    public Translation2d getBackLeftPosition() {
        return m_backLeftPositionMeters;
    }

    public Translation2d getBackRightPosition() {
        return m_backRightPositionMeters;
    }

    public double getMetersPerTick() {
        return m_metersPerTick;
    }

    public Motor.RunMode getRunMode() {
        return m_runMode;
    }
}
