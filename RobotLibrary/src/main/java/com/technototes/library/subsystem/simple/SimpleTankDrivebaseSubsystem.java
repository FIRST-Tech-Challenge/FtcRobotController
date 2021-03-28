package com.technototes.library.subsystem.simple;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.drivebase.TankDrivebaseSubsystem;
@Deprecated
public class SimpleTankDrivebaseSubsystem extends TankDrivebaseSubsystem<Motor<?>> {
    public SimpleTankDrivebaseSubsystem(Motor l, Motor r) {
        super(l, r);
    }
}
