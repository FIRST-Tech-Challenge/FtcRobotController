package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.function.DoubleSupplier;

import teamCode.subsystems.DriveSubsystem;

public class DriveFieldOrientedCommand extends CommandBase
{
    public DriveSubsystem m_driveSubsystem;
    public DoubleSupplier m_leftX;
    public DoubleSupplier m_leftY;
    public DoubleSupplier m_rightX;
    public DoubleSupplier m_rightY;

    public DriveFieldOrientedCommand(DriveSubsystem driveSubsystem, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY)
    {
       this.m_driveSubsystem = driveSubsystem;
       addRequirements(m_driveSubsystem);

       this.m_leftX = leftX;
       this.m_leftY = leftY;
       this.m_rightX = rightX;
       this.m_rightY = rightY;
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
       this.m_driveSubsystem.headingDrive
               (
                       m_leftX.getAsDouble(),
                       m_leftY.getAsDouble(),
                       m_rightX.getAsDouble(),
                       m_rightY.getAsDouble()
               );
    }
}
