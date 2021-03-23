package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.technototes.control.gamepad.Stick;
import com.technototes.library.command.simple.MecanumDriveCommand;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class DriveCommand extends MecanumDriveCommand {
    public DriveCommand(DrivebaseSubsystem subsystem, Stick stick1, Stick stick2) {
        super(subsystem, ()->-stick2.getXAxis(), stick1.getYSupplier(), ()->-stick1.getXAxis());
        setFieldCentric(()->0);

    }

}
