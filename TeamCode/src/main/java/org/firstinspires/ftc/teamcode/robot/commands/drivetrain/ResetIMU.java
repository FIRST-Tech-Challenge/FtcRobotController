package org.firstinspires.ftc.teamcode.robot.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;

public class ResetIMU extends CommandBase
{
    private final DriveSubsystem drive;
    public ResetIMU(DriveSubsystem subsystem) {
        drive = subsystem;
        addRequirements(subsystem);
    }
@Override
    public void initialize() {
        drive.resetIMU();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}