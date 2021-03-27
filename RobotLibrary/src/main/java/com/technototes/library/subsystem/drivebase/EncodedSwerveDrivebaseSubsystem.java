package com.technototes.library.subsystem.drivebase;

import com.technototes.library.hardware.motor.EncodedMotor;
@Deprecated
public class EncodedSwerveDrivebaseSubsystem extends SwerveDrivebaseSubsystem<EncodedMotor<?>, EncodedMotor<?>> {
    public EncodedSwerveDrivebaseSubsystem(EncodedMotor<?>[] d, EncodedMotor<?>[] s) {
        super(d, s);
    }
}
