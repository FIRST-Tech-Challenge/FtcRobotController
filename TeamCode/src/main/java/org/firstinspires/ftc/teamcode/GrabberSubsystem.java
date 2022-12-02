package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GrabberSubsystem extends SubsystemBase {
    private final Servo servo;

    public GrabberSubsystem(final HardwareMap hardwareMap, final String name) {
        servo = hardwareMap.get(Servo.class, name);
    }

    /**
     * Grab
     */
    public void grab() {
        //TODO
        servo.setPosition(1);
    }

    /**
     * Release
     */
    public void release() {
        servo.setPosition(0);
    }
}
