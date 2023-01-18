package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public Servo claw;

    public double OPEN = 0;
    public double CLOSE = 1;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        claw = hwMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
    }

}
