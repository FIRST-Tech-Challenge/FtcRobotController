package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Intake {
    private final Servo clawLeft;
    private final Servo clawRight;
    private final Servo hingeLeft;
    private final Servo hingeRight;
//    private final Servo whacker;
    private boolean isOpen;
//    private boolean whacked;

//    public static final double LEFT_CLOSED_POSITION = 0.03488;
//    public static final double RIGHT_CLOSED_POSITION = 0.7914;
    public static double LARGE_OPEN_POSITION = 0;
    public static double NORMAL_OPEN_POSITION = 0.1;
    public static double CLOSED_POSITION = 0.18;
    public static final double HINGE_MAX_POSITION = 0.65;
    public static final double WHACK_POSITION = 0.2;
    public static final double UNWHACK_POSITION = 0.0;

    public Intake(HardwareMap hardwareMap) {
        clawLeft = hardwareMap.get(Servo.class, "CLAWLEFT");
        clawRight = hardwareMap.get(Servo.class, "CLAWRIGHT");
        hingeLeft = hardwareMap.get(Servo.class, "HINGELEFT");
        hingeRight = hardwareMap.get(Servo.class, "HINGERIGHT");
//        whacker = hardwareMap.get(Servo.class, "WHACKER");

         clawLeft.setDirection(Servo.Direction.FORWARD);
         clawRight.setDirection(Servo.Direction.REVERSE);
         hingeLeft.setDirection(Servo.Direction.FORWARD);
         hingeRight.setDirection(Servo.Direction.REVERSE);
//         whacker.setDirection(Servo.Direction.REVERSE);

//        clawLeft.scaleRange(LEFT_CLOSED_POSITION, LEFT_CLOSED_POSITION + MOVING_DISTANCE);
//        clawRight.scaleRange(RIGHT_CLOSED_POSITION, RIGHT_CLOSED_POSITION + MOVING_DISTANCE);
    }

    public void open() {
        clawLeft.setPosition(NORMAL_OPEN_POSITION);
        clawRight.setPosition(NORMAL_OPEN_POSITION);
        isOpen = true;
    }

    public void largeOpen() {
        clawLeft.setPosition(LARGE_OPEN_POSITION);
        clawRight.setPosition(LARGE_OPEN_POSITION);
        isOpen = true;
    }

    public void close() {
        clawLeft.setPosition(CLOSED_POSITION);
        clawRight.setPosition(CLOSED_POSITION);
        isOpen = false;
    }

    public void hingeTo(double position) {
        position = Math.min(Math.max(0, position), HINGE_MAX_POSITION);
        hingeLeft.setPosition(position);
        hingeRight.setPosition(position);
    }

//    public void whack() {
//        whacker.setPosition(WHACK_POSITION);
//        whacked = true;
//    }
//
//    public void unwhack() {
//        whacker.setPosition(UNWHACK_POSITION);
//        whacked = false;
//    }

    public boolean isOpen() {
        return isOpen;
    }

    public boolean isClosed() {
        return !isOpen;
    }

//    public boolean isWhacked() {
//        return whacked;
//    }
//
//    public boolean isUnwhacked() {
//        return !whacked;
//    }

    public double getLeftClawPosition() {
        return clawLeft.getPosition();
    }

    public double getRightClawPosition() {
        return clawRight.getPosition();
    }

//    public double getWhackerPosition() {
//        return whacker.getPosition();
//    }

//    public void setWhackerPosition(double position) {
//        whacker.setPosition(position);
//    }

    public void setLeftClawPosition(double position) {
        clawLeft.setPosition(position);
    }

    public void setRightClawPosition(double position) {
        clawRight.setPosition(position);
    }
}
