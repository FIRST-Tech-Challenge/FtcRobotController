package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.annotation.Nonnull;

public class Gripper {
    Servo front, back;
    private double position;
    private double currentPosition;
    private double previousServoPosition;
    final double SERVO_POSITION_SIGNIFICANT_DIFFERENCE = 0.01;
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
        return RobotMath.isAbsDiffWithinRange(position, openPos, 0.001);
    }
    public void readPosition() {
        currentPosition = front.getPosition();
    }
    public void writePosition() {
        if (Math.abs(previousServoPosition - position) > SERVO_POSITION_SIGNIFICANT_DIFFERENCE) {
            front.setPosition(position);
            back.setPosition(position);
        }
        previousServoPosition = position;
    }
    public void setPosition(double pos) {
        position = pos;
    }

    @Nonnull
    public String toString() {
        return "Gripper is " + (isOpen() ? "Open" : "Closed");
    }
}
