package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DifferentialV1 {
    public Servo leftServo;
    public AnalogInput leftServoEncoder;
    public Servo rightServo;
    public AnalogInput rightServoEncoder;
    public HardwareMap hwMap = null;
    public void init(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "left_diff");
        rightServo = hardwareMap.get(Servo.class, "right_diff");
        leftServoEncoder = hardwareMap.get(AnalogInput.class, "left_servo_encoder");
        rightServoEncoder = hardwareMap.get(AnalogInput.class, "right_servo_encoder");
        this.hwMap = hardwareMap;
    }

    public void moveUp(double distance){

    }

    public void moveDown(double distance){

    }

    public void rotateLeft(double distance){

    }

    public void rotateRight(double distance){

    }

    public void moveLeftServo(double distance){
        moveLeftServoTo(leftServo.getPosition() + distance);
    }

    public void moveRightServo(double distance){
        moveRightServoTo(rightServo.getPosition() + distance);
    }

    public void moveLeftServoTo(double position){
        leftServo.setPosition(position);
    }

    public void moveRightServoTo(double position){
        rightServo.setPosition(position);
    }

    public double getLeftServoEncoderPosition(){
        return leftServoEncoder.getVoltage() / 3.3 * 360;
    }

    public double getRightServoEncoderPosition(){
        return rightServoEncoder.getVoltage() / 3.3 * 360;
    }

}
