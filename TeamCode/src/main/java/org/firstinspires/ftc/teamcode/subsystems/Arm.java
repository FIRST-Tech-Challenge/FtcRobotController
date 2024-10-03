package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    //Adjustable Constants
    private double ARM_SPEED = 0.001; //Rotation

    //Internal variables
    private Servo armLeft, armRight;


    public Arm(HardwareMap hw){
        this(hw, "armLeft", "armRight");
    }
    public Arm(HardwareMap hw, String nameLeft, String nameRight){
        armLeft = hw.get(Servo.class, nameLeft);
        armRight = hw.get(Servo.class, nameRight);
        armRight.setDirection(Servo.Direction.REVERSE);
    }

    public void changeHeight(double power){
        double newPos = armLeft.getPosition() + (power * ARM_SPEED);
        armLeft.setPosition(newPos);
        armRight.setPosition(newPos);
    }

    public double getPosition(){
        return armLeft.getPosition();
    }

}
