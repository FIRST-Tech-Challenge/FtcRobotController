package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    //220 depoist positoin
    Servo left, right;
    private double ServoPosition;
    private double previousServoPosition;
    private double readServoPosition;
    final double SERVO_POSITION_SIGNIFICANT_DIFFERENCE = 0.01;
    final double DEGREES_FROM_ZERO_TO_ONE = 180;//[-45,135]
    final double ANGLE_IS_ZERO_AT_THIS_SERVO_POS = 0.25;//0.3828;

    public Arm(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "armLeft");
        right = hardwareMap.get(Servo.class, "armRight");

        left.setDirection(Servo.Direction.REVERSE);
        right.setDirection(Servo.Direction.FORWARD);
    }

    public double armAngleCurrent() {
        return servoToDeg(readServoPosition);
    }

    public void goToAngle(double targetDeg) {
        setPosition(degToServo(targetDeg));
    }

    public double servoToDeg(double inputServoPosition) {
        inputServoPosition = RobotMath.maxAndMin(inputServoPosition, 1, 0);
        return (inputServoPosition-ANGLE_IS_ZERO_AT_THIS_SERVO_POS) * DEGREES_FROM_ZERO_TO_ONE;
    }

    public double degToServo(double degrees) {
        degrees = RobotMath.maxAndMin(degrees, 135, -45);
        return (degrees/DEGREES_FROM_ZERO_TO_ONE) + ANGLE_IS_ZERO_AT_THIS_SERVO_POS;
    }

    public void setPosition(double position) {
        ServoPosition = position;
    }

    public void readServoPositions() {
        readServoPosition = left.getPosition();
    }
    public double getCurrentServoPosition() {
        return ServoPosition;
    }
    public void writeServoPositions() {
        if (Math.abs(previousServoPosition - ServoPosition) > SERVO_POSITION_SIGNIFICANT_DIFFERENCE) {
            left.setPosition(ServoPosition);
            right.setPosition(ServoPosition);
        }
        previousServoPosition = ServoPosition;
    }
}
