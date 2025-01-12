package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import teamCode.subsystems.PinPointOdometrySubsystem;

public class PinPointOdometryCommand extends CommandBase
{
    private PinPointOdometrySubsystem m_pinPointOdometrySubsystem;

    public PinPointOdometryCommand(PinPointOdometrySubsystem pinPointOdometrySubsystem)
    {
        this.m_pinPointOdometrySubsystem = pinPointOdometrySubsystem;
        addRequirements(this.m_pinPointOdometrySubsystem);

    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
       m_pinPointOdometrySubsystem.resetOdo();
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
