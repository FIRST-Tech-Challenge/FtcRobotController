package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private Servo servo;
    public Arm(HardwareMap hw){
        servo = hw.get(Servo.class, "arm");
    }
    public void setPosition(double armPower){
        servo.setPosition(armPower);
    }
    public double getPosition(){
        return servo.getPosition();
    }
}
