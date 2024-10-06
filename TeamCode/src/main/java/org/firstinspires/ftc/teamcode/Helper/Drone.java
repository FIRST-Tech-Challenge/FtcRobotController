package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Drone {
    public Servo servo;

    public double Open = 0.92;
    public double Close = 0;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        servo = hwMap.get(Servo.class, "Drone");
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void Unlatched() {
        servo.setPosition(Open);
    }


    public void Latched() {
        servo.setPosition(Close);
    }


}