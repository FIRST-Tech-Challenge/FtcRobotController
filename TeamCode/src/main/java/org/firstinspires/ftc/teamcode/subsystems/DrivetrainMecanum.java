package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivetrainMecanum extends SubsystemBase {
    RevIMU m_gyro;
    Telemetry m_telemetry;
    MecanumDrive m_drivetrain;
    String m_drivemode;


    public DrivetrainMecanum(MotorEx motorBackLeft, MotorEx motorBackRight,
                             MotorEx motorFrontLeft, MotorEx motorFrontRight,
                             Telemetry telemetry, RevIMU gyro, String drivemode) {

        m_gyro = gyro;
        m_telemetry = telemetry;
        m_drivemode = drivemode;

        m_drivetrain = new MecanumDrive(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

    }


    @Override
    public void periodic() {

    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {

        if (m_drivemode.equals("RC")) {
            m_drivetrain.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
        }
        else if (m_drivemode.equals("FC")) {
            m_drivetrain.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, m_gyro.getHeading());
        }
    }
}
