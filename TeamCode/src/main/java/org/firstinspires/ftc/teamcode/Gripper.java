package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.annotation.Nonnull;

public class Gripper {
    Servo front, back;
    final double openPos = 0.05;
    final double closePos = 0.5;

    public Gripper(HardwareMap hardwareMap) {
        front = hardwareMap.get(Servo.class, "gripperFront");
        back = hardwareMap.get(Servo.class, "gripperBack");

        front.setDirection(Servo.Direction.REVERSE);
        back.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(boolean open) {
        if (open) {
            open();
        } else {
            close();
        }
    }
    public void open() {
      setPosition(openPos);
    }
    public void close() {
        setPosition(closePos);
    }

    public void flipFlop() {
        if (isOpen()) {
            close();
        } else {
            open();
        }
    }
    public boolean isOpen() {
        return RobotMath.isAbsDiffWithinRange(front.getPosition(), openPos, 0.001);
    }
    public void setPosition(double position) {
        front.setPosition(position);
        back.setPosition(position);
    }

    @Nonnull
    public String toString() {
        return "Gripper is " + (isOpen() ? "Open" : "Closed");
    }
}
