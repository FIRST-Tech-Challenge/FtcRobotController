package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.IntakePivotSubsystem;

public class IntakePivotCommand extends CommandBase
{
    private static final double m_samplePos = Constants.PivotIntakeConstants.kIntakePivotPickUp;
    private static final double m_specimenPos = Constants.PivotIntakeConstants.kIntakePivotScore;
    private final IntakePivotSubsystem m_intakePivotSubsystem;
    private int m_position;
    private static final int  m_sample = 1;
    private static final int  m_specimen = 0;

    public IntakePivotCommand(IntakePivotSubsystem pivotSubsystem)
    {
        this.m_intakePivotSubsystem = pivotSubsystem;
        m_position = m_specimen;
        addRequirements(this.m_intakePivotSubsystem);
    }

    @Override
    public void initialize()
    {
    }
    @Override
    public void execute()
    {
        if (m_position == m_specimen)
        {
            this.m_intakePivotSubsystem.pivotIntake(m_specimenPos);
            m_position = m_sample;
        }
        else if (m_position == m_sample)
        {
            this.m_intakePivotSubsystem.pivotIntake(m_samplePos);
            m_position = m_specimen;
        }

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