package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.LiftArmSubsystem;
import teamCode.subsystems.SlideArmSubsystem;

public class ResetHomeCommand extends CommandBase
{
    public LiftArmSubsystem m_liftArmSubsystem;
    public SlideArmSubsystem m_slideArmSubsystem;


    public ResetHomeCommand(LiftArmSubsystem liftArmSubsystem, SlideArmSubsystem slideArmSubsystem)
    {
        this.m_liftArmSubsystem = liftArmSubsystem;
        addRequirements(this.m_liftArmSubsystem);
        this.m_slideArmSubsystem = slideArmSubsystem;
        addRequirements(this.m_slideArmSubsystem);
    }

    @Override
    public void initialize()
    {
        this.m_liftArmSubsystem.stop();
        this.m_slideArmSubsystem.stop();
    }

    @Override
    public void execute()
    {
    }

    @Override
    public void end(boolean interrupted)
    {
    }
}
