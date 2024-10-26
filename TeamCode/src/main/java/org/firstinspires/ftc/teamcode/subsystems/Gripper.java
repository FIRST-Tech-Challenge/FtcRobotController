package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private Servo GripperLeft;
    private Servo GripperRight;

    public Gripper(HardwareMap hardwareMap) {
        this.GripperLeft = hardwareMap.get(Servo.class, "Gripper_Left");
        this.GripperLeft.setDirection(Servo.Direction.FORWARD);

        this.GripperRight = hardwareMap.get(Servo.class, "Gripper_Right");
        this.GripperRight.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition1(double position){GripperLeft.setPosition(position);}
    public void setPosition2(double position){GripperRight.setPosition(position);}
    public void getPosition1(){GripperRight.getPosition();}
    public void getPosition2(){GripperLeft.getPosition();}
}