package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.Servo;


public class ServoWrapper {
    private final Servo servo;

    public ServoWrapper(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition() {
        return servo.getPosition();
    }
}