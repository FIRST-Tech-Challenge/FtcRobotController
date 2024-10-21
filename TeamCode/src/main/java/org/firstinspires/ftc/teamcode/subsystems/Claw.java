package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class Claw {
    private Servo servo;
    public Claw(HardwareMap hw){
        servo = hw.get(Servo.class, "claw");
        servo.setPosition(0.5);//number not sure yet.
    }
    public void release(){
        servo.setPosition(0);
    }
    public double getPosition(){
        return servo.getPosition();
    }
}
