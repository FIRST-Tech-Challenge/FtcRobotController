package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive m_drive;

    private final Encoder m_frontLeft, m_frontRight, m_backLeft, m_backRight;

    private final double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem(MotorEx p_frontLeftMotor, MotorEx p_frontRightMotor, MotorEx p_backLeftMotor, MotorEx p_backRightMotor, final double diameter) {
        m_frontLeft = p_frontLeftMotor.encoder;
        m_frontRight = p_frontRightMotor.encoder;
        m_backLeft = p_backLeftMotor.encoder;
        m_backRight = p_backRightMotor.encoder;


        WHEEL_DIAMETER = diameter;

        m_drive = new MecanumDrive(p_frontLeftMotor, p_frontRightMotor, p_backLeftMotor, p_backRightMotor);
    }

    /**
     * Creates a new DriveSubsystem with the hardware map and configuration names.
     */
    public DriveSubsystem(HardwareMap hMap, final String frontLeftMotorName, String frontRightMotorName,
                          String backLeftMotorName, String backRightMotorName,
                          final double diameter) {
        this(new MotorEx(hMap, frontLeftMotorName), new MotorEx(hMap, frontRightMotorName),
                new MotorEx(hMap, backLeftMotorName), new MotorEx(hMap, backRightMotorName), diameter);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param forward the commanded forward movement
     * @param turnSpeed the commanded rotation
     */
    public void drive(double strafe, double forward, double turnSpeed) {
        m_drive.driveRobotCentric(strafe, forward, turnSpeed);
    }

    public double getFrontLeftEncoderVal() {
        return m_frontLeft.getPosition();
    }

    public double getFrontRightEncoderVal() {
        return m_frontRight.getPosition();
    }

    public double getbackRightEncoderDistance() {
        return m_backRight.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getbackLeftEncoderDistance() {
        return m_backLeft.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getBackLeftEncoderVal() {
        return m_backLeft.getPosition();
    }

    public double getBackRightEncoderVal() {
        return m_backRight.getPosition();
    }

    public double getFrontRightEncoderDistance() {
        return m_frontRight.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getFrontLeftEncoderDistance() {
        return m_frontLeft.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }



    public void resetEncoders() {
        m_frontRight.reset();
        m_frontLeft.reset();
        m_backRight.reset();
        m_backLeft.reset();
    }

    public double getAverageEncoderDistance() {
        return (getFrontLeftEncoderDistance() + getFrontRightEncoderDistance() + getbackLeftEncoderDistance() + getbackRightEncoderDistance()) / 4.0;
    }

}
