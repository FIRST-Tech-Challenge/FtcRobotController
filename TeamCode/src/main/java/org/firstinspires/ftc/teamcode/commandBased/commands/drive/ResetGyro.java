package org.firstinspires.ftc.teamcode.commandBased.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;

public class ResetGyro extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    public ResetGyro(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
