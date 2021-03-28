package com.technototes.library.subsystem.simple;

import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.EncodedMotorGroup;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.motor.EncodedMotorSubsystem;
@Deprecated
public class SimpleEncodedMotorSubsystem extends EncodedMotorSubsystem {
    public SimpleEncodedMotorSubsystem(EncodedMotor<?> m1, Motor<?>... m2) {
        super(new EncodedMotorGroup(m1, m2));
    }
}
