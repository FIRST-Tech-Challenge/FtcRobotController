package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.LiftArmSubsystem;

public class ArmFudgeFactorDownCommand extends CommandBase
{
    private LiftArmSubsystem m_liftArmSubsystem;

    public int m_lift;

    public ArmFudgeFactorDownCommand(LiftArmSubsystem liftArmSubsystem)
    {
        this.m_liftArmSubsystem = liftArmSubsystem;

        addRequirements(m_liftArmSubsystem);

        this.m_lift = -50;
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_liftArmSubsystem.fudgeFactor(m_lift);
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
