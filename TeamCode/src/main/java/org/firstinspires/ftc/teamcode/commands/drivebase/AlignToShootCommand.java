package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.technototes.library.command.Command;
import com.technototes.library.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class AlignToShootCommand extends Command {
    public DrivebaseSubsystem drivebaseSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public AlignToShootCommand(DrivebaseSubsystem drive, ShooterSubsystem shoot){
        addRequirements(drive, shoot);
        drivebaseSubsystem = drive;
        shooterSubsystem = shoot;
    }

    @Override
    public void init() {
        //TODO calculate changes needed
    }

    @Override
    public void execute() {
        //TODO make the changes
    }

    @Override
    public boolean isFinished() {
        //TODO check if changes are made
        return super.isFinished();
    }
}
