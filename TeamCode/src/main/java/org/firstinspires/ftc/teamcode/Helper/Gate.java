package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Gate {
    public Servo servo;

    public double Open = 0.68;
    public double Close = 0.5;

    public double Middle = 0.55;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        servo = hwMap.get(Servo.class, "Gate");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void open() {
        servo.setPosition(Open);
    }
    public void middle(){
        servo.setPosition(Middle);
    }

    public void close() {
        servo.setPosition(Close);
    }


}