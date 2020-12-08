package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.technototes.control.gamepad.Stick;
import com.technototes.library.command.simple.MecanumDriveCommand;
import com.technototes.library.subsystem.drivebase.MecanumDrivebaseSubsystem;

public class DriveCommand extends MecanumDriveCommand {
    public DriveCommand(MecanumDrivebaseSubsystem subsystem, Stick stick1, Stick stick2) {
        super(subsystem, stick1, stick2);
    }
}
