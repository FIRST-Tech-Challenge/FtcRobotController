package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import teamCode.subsystems.IntakeWheelSubsystem;

public class SpinIntakeAutoCommand extends CommandBase
{
    private final IntakeWheelSubsystem m_intakeWheelSubsystem;
    private int m_spin;

    public SpinIntakeAutoCommand(IntakeWheelSubsystem wheel, int spin)
    {
        this.m_intakeWheelSubsystem = wheel;
        this.m_spin = spin;
        addRequirements(this.m_intakeWheelSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_intakeWheelSubsystem.spinIntake(m_spin);
    }

    public void end(boolean interrupt)
    {
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
