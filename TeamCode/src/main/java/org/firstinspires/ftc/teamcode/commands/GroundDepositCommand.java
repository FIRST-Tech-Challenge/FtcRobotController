package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class GroundDepositCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private Trajectory m_trajectorySpot1;
    private Trajectory m_trajectorySpot0;
    private Trajectory m_trajectorySpot2;

    public GroundDepositCommand(DriveSubsystem driveSubsystem,
                          VisionSubsystem visionSubsystem,
                          String parkingTrajectory0,
                          String parkingTrajectory1,
                          String parkingTrajectory2)
    {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_trajectorySpot2 = AssetsTrajectoryManager.load(parkingTrajectory2);
        m_trajectorySpot1 = AssetsTrajectoryManager.load(parkingTrajectory1);
        m_trajectorySpot0 = AssetsTrajectoryManager.load(parkingTrajectory0);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        {
            if (m_visionSubsystem.getLocation() == 0)//
            {
                m_driveSubsystem.followTrajectoryAsync(m_trajectorySpot0);
            } else if (m_visionSubsystem.getLocation() == 1) //
            {
                m_driveSubsystem.followTrajectoryAsync(m_trajectorySpot1);
            } else //
            {
                m_driveSubsystem.followTrajectoryAsync(m_trajectorySpot2);
            }
        }
    }

    @Override
    public void execute()
    {
        m_driveSubsystem.update();
    }

    @Override
    public boolean isFinished()
    {
        return Thread.currentThread().isInterrupted() || !m_driveSubsystem.isBusy();
    }

    @Override
    public void end(boolean interrupted)
    {
        if(interrupted)
        {
            m_driveSubsystem.stop();
        }
    }
}
