package com.technototes.library.subsystem.simple;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.hardware.servo.ServoGroup;
import com.technototes.library.subsystem.servo.ServoSubsystem;
@Deprecated
public class SimpleServoSubsystem extends ServoSubsystem {
    public SimpleServoSubsystem(Servo s1, Servo... s2) {
        super(new ServoGroup(s1, s2));
    }

    public SimpleServoSubsystem(Servo s1) {
        super(s1);
    }
}
