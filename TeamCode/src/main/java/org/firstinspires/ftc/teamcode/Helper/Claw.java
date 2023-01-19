package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public Servo servo;

    public double OPEN = 0;
    public double CLOSE = 1;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        servo = hwMap.get(Servo.class, "claw");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void open() {
        servo.setPosition(OPEN);
    }

    public void close() {
        servo.setPosition(CLOSE);
    }

    public double getPosition() {
        double position =  servo.getPosition();
        return position;
    }


}
