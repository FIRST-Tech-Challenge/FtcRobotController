package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.OldDrivebaseSubsystem;

public class TurboSpeedCommand extends Command {
    public OldDrivebaseSubsystem subsystem;
    public TurboSpeedCommand(OldDrivebaseSubsystem sub){
        //dont add requirements to the subsystem because we dont want to stop other commands when we change speed
        subsystem = sub;
    }

    @Override
    public void execute() {
        subsystem.setDriveSpeed(OldDrivebaseSubsystem.DriveSpeed.TURBO);
    }
}
