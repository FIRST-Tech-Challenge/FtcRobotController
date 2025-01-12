package teamCode.autoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import teamCode.Logic;
import teamCode.autoSubsystems.AutoDriveSubsystem;
public class AutoDriveCommand extends CommandBase
{
    public AutoDriveSubsystem m_autoDriveSubsystem;
    public int m_fL;
    public int m_fR;
    public int m_bL;
    public int m_bR;
    public AutoDriveCommand(AutoDriveSubsystem autoDriveSubsystem, int fL, int fR, int bL, int bR)
    {
        this.m_autoDriveSubsystem = autoDriveSubsystem;
        this.m_fL = fL;
        this.m_fR = fR;
        this.m_bL = bL;
        this.m_bR = bR;
        addRequirements(this.m_autoDriveSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_autoDriveSubsystem.driveRobot(this.m_fL, this.m_fR, this.m_bL, this.m_bR);
    }

    @Override
    public void end(boolean interrupted)
    {
//       this.m_autoDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished()
    {
        return Logic.OpModeType.opMode.equals("Sting-Ray Auto")
                && this.m_autoDriveSubsystem.atTarget(this.m_fL);
    }

}