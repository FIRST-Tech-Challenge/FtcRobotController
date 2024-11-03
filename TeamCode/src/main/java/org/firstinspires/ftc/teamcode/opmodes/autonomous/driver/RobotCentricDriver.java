package org.firstinspires.ftc.teamcode.opmodes.autonomous.driver;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoFourWheelMecanumDriveTrain;

/**
 * Robot centric driver that will invoke {@link MecanumDrive#driveRobotCentric(double, double, double)}
 * to drive the robot
 */
public class RobotCentricDriver extends AutoRobotDriver {

    public RobotCentricDriver(AutoFourWheelMecanumDriveTrain driveTrain) {
        super(driveTrain);
    }

    @Override
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        driveTrain.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }
}
