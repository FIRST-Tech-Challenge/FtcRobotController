package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Deprecated
public class Grabber {
    private final Servo grabberServo;
    private boolean isOpen;

    private static final float openPosition = 0.5f;
    private static final float closePosition = .8f;


    public Grabber(HardwareMap hardwareMap, String servoName) {
        grabberServo = hardwareMap.servo.get(servoName);
    }

    public void open() {
        grabberServo.setPosition(openPosition);
        isOpen = true;
    }

    public void close() {
        grabberServo.setPosition(closePosition);
        isOpen = false;
    }

    public boolean isOpen() {
        return isOpen;
    }

    public boolean isClosed() {
        return !isOpen;
    }
}
