package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {

    private final DriveSubsystem m_drive;
    private final double m_distance;
    private final double m_speed;

    /**
     * Creates a new DriveDistance.
     *
     * @param inches The number of inches the robot will drive
     * @param speed  The speed at which the robot will drive
     * @param drive  The drive subsystem on which this command will run
     */
    public DriveDistance(double inches, double speed, DriveSubsystem drive) {
        m_distance = inches;
        m_speed = speed;
        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_drive.resetEncoders();
        m_drive.drive(0, 1, 0);
    }

    @Override
    public void execute() {
        m_drive.drive(0, 1, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0);
    }


    @Override
    public boolean isFinished() {
        return false;
    }

}
