package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.configNames;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

public class arm {
    robotConfig r;

    public enum armPos{
        FRONT(0),
        BACK(1);

        private armPos(double position){
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return position;
        }
    }

    Servo arm;
    private double armPos;

    public arm(robotConfig r) {
        this.r = r;
        arm = r.hardwareMap.get(Servo.class,configNames.arm);
        arm.scaleRange(robotConstants.armBack, robotConstants.armFront);
        freeTargetPosition(0);
    }

    public void freeTargetPosition(double targetPos){
        armPos = targetPos;
    }

    public void presetTargetPosition(armPos armPos){
        this.armPos = armPos.getPosition();
    }

    public void update(){
        arm.setPosition(armPos);
    }
}
