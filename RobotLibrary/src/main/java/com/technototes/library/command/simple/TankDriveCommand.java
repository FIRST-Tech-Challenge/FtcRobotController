package com.technototes.library.command.simple;

import com.technototes.library.command.Command;
import com.technototes.library.control.gamepad.old.OldCommandGamepad;
import com.technototes.library.subsystem.drivebase.TankDrivebaseSubsystem;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends Command {
    public TankDrivebaseSubsystem subsystem;
    public DoubleSupplier xv, yv;
    public TankDriveCommand(TankDrivebaseSubsystem s, DoubleSupplier x, DoubleSupplier y){
        addRequirements(s);
        subsystem = s;
        xv = x;
        yv = y;
    }
    public TankDriveCommand(TankDrivebaseSubsystem s, OldCommandGamepad.Stick st){
        new TankDriveCommand(s, st.x, st.y);
    }
    @Override
    public void execute() {
        subsystem.arcadeDrive(yv.getAsDouble(), yv.getAsDouble());
    }
}
