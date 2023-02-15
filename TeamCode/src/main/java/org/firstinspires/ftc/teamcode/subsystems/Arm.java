package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Arm {
    RobotConfig r;

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

    public Arm(RobotConfig r) {
        this.r = r;
        arm = r.hardwareMap.get(Servo.class, ConfigNames.arm);
        arm.scaleRange(RobotConstants.armBack, RobotConstants.armFront);
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
