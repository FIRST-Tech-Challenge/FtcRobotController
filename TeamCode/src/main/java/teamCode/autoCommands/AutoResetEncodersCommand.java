package teamCode.autoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.autoSubsystems.AutoDriveSubsystem;

public class AutoResetEncodersCommand extends CommandBase
{
    public AutoDriveSubsystem m_autoDriveSubsystem;
    public AutoResetEncodersCommand(AutoDriveSubsystem autoDriveSubsystem)
    {
        this.m_autoDriveSubsystem = autoDriveSubsystem;
        addRequirements(this.m_autoDriveSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_autoDriveSubsystem.stop();
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}