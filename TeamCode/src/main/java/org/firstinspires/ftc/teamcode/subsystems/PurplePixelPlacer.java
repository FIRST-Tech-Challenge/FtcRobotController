package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.rustlib.commandsystem.Subsystem;
import org.rustlib.rustboard.Rustboard;

public class PurplePixelPlacer extends Subsystem {

    private final Servo servo;

    public PurplePixelPlacer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "purplePixelPlacer");
    }

    public void place() {
        servo.setPosition(Rustboard.getDoubleValue("purple place", 0.95));
    }

    public void retract() {
        servo.setPosition(Rustboard.getDoubleValue("purple retract", 0.6));
    }
}
