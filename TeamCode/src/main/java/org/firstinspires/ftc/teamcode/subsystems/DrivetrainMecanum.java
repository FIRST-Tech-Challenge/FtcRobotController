package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils;

public class DrivetrainMecanum extends SubsystemBase {
    RevIMU m_gyro;
    Telemetry m_telemetry;
    MecanumDrive m_drivetrain;
    String m_drivemode;
    BNO055IMU m_imu;
    MotorEx m_motorFrontLeft, m_motorFrontRight, m_motorBackLeft, m_motorBackRight;
    Telemetry.Item tAcceleration, tHeading, tMotorSpeeds;
    Utils.RampRate strafeRate, forwardRate, turnRate;

    static final Double MAX_RAMP = 0.01;





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

        strafeRate = new Utils.RampRate(MAX_RAMP);
        forwardRate = new Utils.RampRate(MAX_RAMP);
        turnRate = new Utils.RampRate(MAX_RAMP);


        m_drivetrain = new MecanumDrive(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        m_telemetry.addLine("Drivetrain Initialized");

    }


    @Override
    public void periodic() {
        m_telemetry.addData("Acceleration", m_imu.getAcceleration());
        m_telemetry.addData("Heading", m_gyro.getHeading());
        m_telemetry.addData("Motor Speeds","backLeft: %.2f, backRight: %.2f, frontLeft: %.2f, frontRight: %.2f",
                m_motorBackLeft.get(), m_motorBackRight.get(), m_motorFrontLeft.get(), m_motorFrontRight.get());
    }

    public void driveWithLimiter(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        /*

        Applies a stick limiter to smooth ramp rate. This is not working currently

        Takes input from the gamepad (in the command) and runs the motors
        If the mode is robot centric, the sticks will work as if you're on the robot
        facing forward all the time.

        If the mode is field centric, the sticks will always move the robot in the direction
        you point them. As if you were looking at the field from the top down
         */
        strafeSpeed = strafeRate.update(strafeSpeed);
        forwardSpeed = forwardRate.update(forwardSpeed);
        turnSpeed = turnRate.update(turnSpeed);


        if (m_drivemode.equals("RC")) {
            m_drivetrain.driveRobotCentric(strafeSpeed,forwardSpeed,
                    turnSpeed);
        }
        else if (m_drivemode.equals("FC")) {
            m_drivetrain.driveFieldCentric(strafeSpeed,
                    forwardSpeed,
                    turnSpeed, m_gyro.getHeading());
        }
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        /* Takes input from the gamepad (in the command) and runs the motors
        If the mode is robot centric, the sticks will work as if you're on the robot
        facing forward all the time.

        If the mode is field centric, the sticks will always move the robot in the direction
        you point them. As if you were looking at the field from the top down
         */

        if (m_drivemode.equals("RC")) {
            m_drivetrain.driveRobotCentric(strafeSpeed,forwardSpeed,
                    turnSpeed);
        }
        else if (m_drivemode.equals("FC")) {
            m_drivetrain.driveFieldCentric(strafeSpeed,
                    forwardSpeed,
                    turnSpeed, m_gyro.getHeading());
        }
    }
}
