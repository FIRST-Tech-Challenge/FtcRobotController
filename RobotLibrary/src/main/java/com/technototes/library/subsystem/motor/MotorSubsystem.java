package com.technototes.library.subsystem.motor;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.subsystem.SpeedSubsystem;

public class MotorSubsystem<T extends Motor> extends Subsystem<T> implements SpeedSubsystem {
    public MotorSubsystem(T... d) {
        super(d);
    }

    @Override
    public double getSpeed() {
        return devices[0].getSpeed();
    }
    @Override
    public void setSpeed(double val) {
        for (T m : devices) {
            m.setSpeed(val);
        }
    }
}
