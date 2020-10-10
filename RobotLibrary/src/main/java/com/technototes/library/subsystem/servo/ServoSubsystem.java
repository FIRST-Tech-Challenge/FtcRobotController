package com.technototes.library.subsystem.servo;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.PID;
import com.technototes.library.subsystem.Subsystem;

public class ServoSubsystem<T extends Servo> extends Subsystem<T> {
    public ServoSubsystem(T... d) {
        super(d);
    }

    public void setPosition(double val) {
        for (T m : devices) {
            m.setPosition(val);
        }
    }

}
