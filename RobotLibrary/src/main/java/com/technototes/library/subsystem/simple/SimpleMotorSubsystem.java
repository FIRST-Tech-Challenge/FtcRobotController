package com.technototes.library.subsystem.simple;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.motor.MotorGroup;
import com.technototes.library.subsystem.motor.MotorSubsystem;
@Deprecated
public class SimpleMotorSubsystem extends MotorSubsystem<Motor> {
    public SimpleMotorSubsystem(Motor m1, Motor... m2) {
        super(new MotorGroup(m1, m2));
    }
}
