package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slider {
    public Servo servo;
    HardwareMap hwMap = null;

    public double Retract = 0;
    public double Extend = 1;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        servo = hwMap.get(Servo.class, "Slider");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void retract() {
        servo.setPosition(Retract);
    }

    public void extend() {servo.setPosition(Extend); }

    public void justExtend() {servo.setPosition(0.3*Extend); }

    public void halfExtend() {servo.setPosition(0.5*Extend);}

}