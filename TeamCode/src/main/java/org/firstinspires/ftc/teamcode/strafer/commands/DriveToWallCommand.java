package org.firstinspires.ftc.teamcode.strafer.commands;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.strafer.subsystems.DrivebaseSubsystem;

public class DriveToWallCommand extends Command {
    public DrivebaseSubsystem drivebase;
    public DriveToWallCommand(DrivebaseSubsystem s){
        drivebase = s;
        addRequirements(s);
    }

    @Override
    public void execute() {
        drivebase.drive(1, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return drivebase.getDistance() < 5;
    }

    @Override
    public void end(boolean cancel) {
        drivebase.stop();
    }
}
