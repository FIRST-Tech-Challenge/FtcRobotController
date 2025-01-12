package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.ClimbArmSubsystem;

public class ClimbArmDownCommand extends CommandBase
{
    private ClimbArmSubsystem m_climbArmSubsystem;

    public ClimbArmDownCommand(ClimbArmSubsystem climbArmSubsystem)
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
        if (!m_climbArmSubsystem.atTarget(0))
        {
            this.m_climbArmSubsystem.climb(Constants.ClimbArmConstants.kClimberArmDown);
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
