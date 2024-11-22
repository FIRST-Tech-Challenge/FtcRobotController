package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private Servo armLeft;
    private Servo armRight;
    public Arm(HardwareMap hw, String armLeftName, String armRightName){

        armLeft = hw.get(Servo.class, armLeftName);
        armRight = hw.get(Servo.class, armRightName);
        armRight.setDirection(Servo.Direction.FORWARD);
        armLeft.setDirection(Servo.Direction.REVERSE);

    }
    public void setPosition(double armPower){
        double offset = 0.6;
        armLeft.setPosition(armPower + offset);
        armRight.setPosition(armPower + offset);

    }
    public double getLeftPosition(){
        return armLeft.getPosition();
    }

    public double getRightPosition()
    {
        return armRight.getPosition();
    }




}
