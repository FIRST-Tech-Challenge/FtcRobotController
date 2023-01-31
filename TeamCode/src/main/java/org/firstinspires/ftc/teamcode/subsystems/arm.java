package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.configNames;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

public class arm {
    robotConfig r;

    double jump = 100.68;

    Servo arm;
    private double armPos;

    public arm(robotConfig r) {
        this.r = r;
        arm = r.hardwareMap.get(Servo.class,configNames.arm);
        arm.scaleRange(robotConstants.armBack, robotConstants.armFront);
        setPos(0);
    }

    public void setPos(double targetPos){
        armPos = targetPos;
    }

    public void update(){
        arm.setPosition(armPos);
    }

    public double returnJump(){
        return jump;
    }

    public void setJump(double value){
        jump = value;
    }
}
