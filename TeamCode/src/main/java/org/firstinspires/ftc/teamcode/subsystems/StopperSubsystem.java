package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StopperSubsystem {

    private static final double ON_POS = 0.3266;
    private static final double OFF_POS = 0.5;
    private final Servo servo;

    public StopperSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "stopper");
        off(); // Default position on initialization
    }

    public void on() {
        servo.setPosition(ON_POS);
    }

    public void off() {
        servo.setPosition(OFF_POS);
    }
}