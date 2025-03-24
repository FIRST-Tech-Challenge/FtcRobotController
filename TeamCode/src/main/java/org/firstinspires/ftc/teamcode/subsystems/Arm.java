package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private Servo backArm;
    private Servo frontArm;
    public Arm(HardwareMap hardwareMap, String frontArmName, String backArmName)
    {
        backArm = hardwareMap.get(Servo.class, backArmName);
        frontArm = hardwareMap.get(Servo.class, frontArmName);

    }
    public void setPosition(double pos){
        double liftPosition = 0.5;
        backArm.setPosition(liftPosition + pos);
        frontArm.setPosition(liftPosition + pos);
    }
    public double getFrontArm(){
        return frontArm.getPosition();
    }
    public double getBackArm(){
        return backArm.getPosition();
    }
}
