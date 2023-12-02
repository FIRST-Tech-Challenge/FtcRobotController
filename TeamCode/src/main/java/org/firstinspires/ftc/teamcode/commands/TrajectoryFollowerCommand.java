package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.roadrunner.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TrajectoryFollowerCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final Trajectory m_trajectory;

    public TrajectoryFollowerCommand(DriveSubsystem driveSubsystem, String trajectoryName)
    {
        m_driveSubsystem = driveSubsystem;
        m_trajectory = AssetsTrajectoryManager.load(trajectoryName);
        addRequirements(driveSubsystem);
    }
    public TrajectoryFollowerCommand(DriveSubsystem driveSubsystem, String trajectoryName, double maxVel, double maxAccel)
    {
        m_driveSubsystem = driveSubsystem;
        m_trajectory = AssetsTrajectoryManager.load(trajectoryName, maxVel, maxAccel);
        addRequirements(driveSubsystem);
    }


    @Override
    public void initialize()
    {
//        Pose2d initialPose = m_trajectory.get(0.0);
//        m_driveSubsystem.setPoseEstimate(initialPose);
        m_driveSubsystem.followTrajectoryAsync(m_trajectory);
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
