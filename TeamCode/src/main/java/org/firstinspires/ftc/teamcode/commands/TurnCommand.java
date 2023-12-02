package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TurnCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final double m_angle; //In Radians

    public TurnCommand(DriveSubsystem driveSubsystem, double angle)
    {
        m_driveSubsystem = driveSubsystem;
        m_angle = angle; //In Radians
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {
        m_driveSubsystem.turnAsync(m_angle);
    }

    @Override
    public void execute()
    {
        m_driveSubsystem.update();
    }

    @Override
    public void end(boolean interrupted)
    {
        if(interrupted)
        {
            m_driveSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished()
    {
        return Thread.currentThread().isInterrupted() || !m_driveSubsystem.isBusy();
    }
}
