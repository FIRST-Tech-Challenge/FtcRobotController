package org.firstinspires.ftc.teamcode.opmodes.autonomous.driver;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoFourWheelMecanumDriveTrain;

/**
 * Field centric driver that will invoke {@link MecanumDrive#driveFieldCentric(double, double, double, double)}
 * to drive the robot
 */
public class FieldCentricDriver extends AutoRobotDriver {

    public FieldCentricDriver(AutoFourWheelMecanumDriveTrain driveTrain) {
        super(driveTrain);
    }

    @Override
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        driveTrain.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }
}
