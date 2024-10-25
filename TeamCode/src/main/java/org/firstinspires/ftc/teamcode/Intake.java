package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final Servo leftServo;
    private final Servo rightServo;
    private boolean isOpen;

    public static final double OPEN_POSITION = 0.2;
    public static final double CLOSED_POSITION = 0.05;

    public Intake(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "CLAWLEFT");
        rightServo = hardwareMap.get(Servo.class, "CLAWRIGHT");

         leftServo.setDirection(Servo.Direction.FORWARD);
         rightServo.setDirection(Servo.Direction.REVERSE);

        leftServo.scaleRange(CLOSED_POSITION - 0.02, OPEN_POSITION - 0.02);
        rightServo.scaleRange(CLOSED_POSITION, OPEN_POSITION);
    }

    public void open() {
        leftServo.setPosition(1.0);
        rightServo.setPosition(1.0);
        isOpen = true;
    }

    public void close() {
        leftServo.setPosition(0.0);
        rightServo.setPosition(0.0);
        isOpen = false;
    }

    public boolean isOpen() {
        return isOpen;
    }

    public boolean isClosed() {
        return !isOpen;
    }

    public double getLeftPosition() {
        return leftServo.getPosition();
    }

    public double getRightPosition() {
        return rightServo.getPosition();
    }
}
