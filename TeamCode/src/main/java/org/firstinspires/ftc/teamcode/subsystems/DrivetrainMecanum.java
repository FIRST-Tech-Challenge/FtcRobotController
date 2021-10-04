package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivetrainMecanum extends SubsystemBase {
    RevIMU m_gyro;
    Telemetry m_telemetry;
    MecanumDrive m_drivetrain;
    String m_drivemode;
    BNO055IMU m_imu;
    MotorEx m_motorFrontLeft, m_motorFrontRight, m_motorBackLeft, m_motorBackRight;
    Telemetry.Item tAcceleration, tHeading, tMotorSpeeds;




    public DrivetrainMecanum(MotorEx motorBackLeft, MotorEx motorBackRight,
                             MotorEx motorFrontLeft, MotorEx motorFrontRight,
                             Telemetry telemetry, RevIMU gyro, String drivemode,
                             BNO055IMU imu) {

        m_gyro = gyro;
        m_telemetry = telemetry;
        m_drivemode = drivemode;
        m_imu = imu;
        m_motorFrontLeft = motorFrontLeft;
        m_motorFrontRight = motorFrontRight;
        m_motorBackLeft = motorBackLeft;
        m_motorBackRight = motorBackRight;

        m_drivetrain = new MecanumDrive(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        m_telemetry.addLine("Drivetrain Initialized");

        tAcceleration = m_telemetry.addData("Acceleration", 0);
        tHeading = m_telemetry.addData("Heading", 0);
        tMotorSpeeds = m_telemetry.addData("Motor Speeds", "Stopped");

    }


    @Override
    public void periodic() {
        tAcceleration.setValue(m_imu.getAcceleration());
        tHeading.setValue(m_gyro.getHeading());
        tMotorSpeeds.setValue("backLeft: %.2f, backRight: %.2f, frontLeft: %.2f, frontRight: %.2f",
                m_motorBackLeft.get(), m_motorBackRight.get(), m_motorFrontLeft.get(), m_motorFrontRight.get());

        m_telemetry.update();

    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {

        Double scale = 0.75;

        if (m_drivemode.equals("RC")) {
//            m_drivetrain.driveRobotCentric(stickLimiter(strafeSpeed, 1.5),
//                    stickLimiter(forwardSpeed, 3.0),
//                    stickLimiter(turnSpeed, 1.5));
            m_drivetrain.driveRobotCentric(strafeSpeed * scale,forwardSpeed * scale,
                    turnSpeed * scale);
        }
        else if (m_drivemode.equals("FC")) {
            m_drivetrain.driveFieldCentric(stickLimiter(strafeSpeed, 3.0),
                    stickLimiter(forwardSpeed, 3.0),
                    stickLimiter(turnSpeed, 3.0), m_gyro.getHeading());
        }
    }

    public Double stickLimiter(Double magnitude, Double scale) {
        return Math.pow(magnitude, scale);
    }
}
