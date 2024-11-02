package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private CRServo servo;
    public Arm(HardwareMap hw){

        servo = hw.get(CRServo.class, "arm");
    }
//    public void setPosition(double armPower){
//        servo.setPosition(armPower);
//
//    }
//    public double getPosition(){
//        return servo.getPosition();
//    }

    public void setPower(double armPower)
    {
        servo.setPower(armPower);
    }
}
