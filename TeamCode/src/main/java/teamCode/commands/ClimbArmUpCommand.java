package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.ClimbArmSubsystem;

public class ClimbArmUpCommand extends CommandBase
{
    private ClimbArmSubsystem m_climbArmSubsystem;

    public ClimbArmUpCommand(ClimbArmSubsystem climbArmSubsystem)
    {
        this.m_climbArmSubsystem = climbArmSubsystem;

        addRequirements(m_climbArmSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        if (!m_climbArmSubsystem.atTarget(3000))
        {
            this.m_climbArmSubsystem.climb(Constants.ClimbArmConstants.kClimberArmUp);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
//        this.m_climbArmSubsystem.stop();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
