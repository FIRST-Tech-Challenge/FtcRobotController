package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.LiftArmSubsystem;

public class ScoreSpecimenCommand extends CommandBase
{
    private LiftArmSubsystem m_liftArmSubsystem;

    public int m_lift;

    public ScoreSpecimenCommand(LiftArmSubsystem liftArmSubsystem)
    {
        this.m_liftArmSubsystem = liftArmSubsystem;

        addRequirements(m_liftArmSubsystem);

        this.m_lift = 200;
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_liftArmSubsystem.scoreSpecimen(m_lift);
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
