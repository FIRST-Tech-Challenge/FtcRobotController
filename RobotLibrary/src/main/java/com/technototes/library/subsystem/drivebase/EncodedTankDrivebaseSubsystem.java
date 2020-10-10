package com.technototes.library.subsystem.drivebase;

import com.technototes.library.hardware.motor.EncodedMotor;

public class EncodedTankDrivebaseSubsystem extends TankDrivebaseSubsystem<EncodedMotor> {
    public EncodedTankDrivebaseSubsystem(EncodedMotor l, EncodedMotor r) {
        super(l, r);
    }
}
