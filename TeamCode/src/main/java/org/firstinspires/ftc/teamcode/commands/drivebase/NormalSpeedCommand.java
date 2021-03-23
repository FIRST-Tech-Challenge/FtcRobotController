package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class NormalSpeedCommand extends Command {
    public DrivebaseSubsystem subsystem;
    public NormalSpeedCommand(DrivebaseSubsystem sub){
        //dont add requirements to the subsystem because we dont want to stop other commands when we change speed
        subsystem = sub;
    }

    @Override
    public void execute() {
        subsystem.setDriveSpeed(DrivebaseSubsystem.DriveSpeed.NORMAL);
    }
}
