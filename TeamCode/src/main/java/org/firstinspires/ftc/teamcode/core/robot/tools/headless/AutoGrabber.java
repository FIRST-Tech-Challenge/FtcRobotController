package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

public class AutoGrabber {
    private final Servo leftServo;
    private final Servo rightServo;
    private boolean open = true;
    public AutoGrabber(@NonNull HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftTse");
        leftServo.resetDeviceConfigurationForOpMode();
        rightServo = hardwareMap.get(Servo.class, "rightTse");
        rightServo.resetDeviceConfigurationForOpMode();
    }

    public void close() {
        leftServo.setPosition(0.42);
        rightServo.setPosition(0.59);
        if (open) {
            open = false;
        }
    }
    public void open() {
        leftServo.setPosition(0);
        rightServo.setPosition(1);
        if (!open) {
            open = true;
        }
    }
    public void toggle() {
        if (open) {
            close();
        } else {
            open();
        }
    }
}
