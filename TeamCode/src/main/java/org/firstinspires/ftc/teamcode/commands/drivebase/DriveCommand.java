package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.technototes.control.gamepad.Stick;
import com.technototes.library.command.simple.MecanumDriveCommand;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class DriveCommand extends MecanumDriveCommand {
    public DriveCommand(DrivebaseSubsystem subsystem, Stick stick1, Stick stick2) {
        super(subsystem, ()->Math.abs(stick1.getXAxis()) > 0.05 ? -stick1.getXAxis() : 0,
                ()->Math.abs(stick1.getYAxis()) > 0.05 ? -stick1.getYAxis() : 0,
                ()->Math.abs(stick2.getXAxis()) > 0.05 ? -stick2.getXAxis() : 0);
        setFieldCentric(subsystem.imu);

    }

}
