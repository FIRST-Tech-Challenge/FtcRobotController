package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final Servo leftServo;
    private final Servo rightServo;
    private final Servo whacker;
    private boolean isOpen;
    private boolean whacked;

    public static final double MOVING_DISTANCE = 0.058;
    public static final double LEFT_CLOSED_POSITION = 0.03488;
    public static final double RIGHT_CLOSED_POSITION = 0.7914;
    public static final double WHACK_POSITION = 0.2;
    public static final double UNWHACK_POSITION = 0.0;

    public Intake(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "CLAWLEFT");
        rightServo = hardwareMap.get(Servo.class, "CLAWRIGHT");
        whacker = hardwareMap.get(Servo.class, "WHACKER");

         leftServo.setDirection(Servo.Direction.FORWARD);
         rightServo.setDirection(Servo.Direction.REVERSE);
         whacker.setDirection(Servo.Direction.REVERSE);

//        leftServo.scaleRange(LEFT_CLOSED_POSITION, LEFT_CLOSED_POSITION + MOVING_DISTANCE);
//        rightServo.scaleRange(RIGHT_CLOSED_POSITION, RIGHT_CLOSED_POSITION + MOVING_DISTANCE);
    }

    public void open() {
        leftServo.setPosition(LEFT_CLOSED_POSITION + MOVING_DISTANCE);
        rightServo.setPosition(RIGHT_CLOSED_POSITION + MOVING_DISTANCE);
        isOpen = true;
    }

    public void close() {
        leftServo.setPosition(LEFT_CLOSED_POSITION);
        rightServo.setPosition(RIGHT_CLOSED_POSITION);
        isOpen = false;
    }

    public void whack() {
        whacker.setPosition(WHACK_POSITION);
        whacked = true;
    }

    public void unwhack() {
        whacker.setPosition(UNWHACK_POSITION);
        whacked = false;
    }

    public boolean isOpen() {
        return isOpen;
    }

    public boolean isClosed() {
        return !isOpen;
    }

    public boolean isWhacked() {
        return whacked;
    }

    public boolean isUnwhacked() {
        return !whacked;
    }

    public double getLeftPosition() {
        return leftServo.getPosition();
    }

    public double getRightPosition() {
        return rightServo.getPosition();
    }

    public double getWhackerPosition() {
        return whacker.getPosition();
    }

    public void setWhackerPosition(double position) {
        whacker.setPosition(position);
    }

    public void setLeftPosition(double position) {
        leftServo.setPosition(position);
    }

    public void setRightPosition(double position) {
        rightServo.setPosition(position);
    }
}
