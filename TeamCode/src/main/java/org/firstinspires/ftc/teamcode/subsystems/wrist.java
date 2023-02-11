package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.configNames;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

public class wrist {

    robotConfig r;

    public enum wristPos{
        FRONT(1),
        BACK(0);

        private wristPos(double position){
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return position;
        }
    }

    Servo wrist;
    private double wristPos;

    public wrist(robotConfig r) {
        this.r = r;

        wrist = r.hardwareMap.get(Servo.class, configNames.wrist);
        wrist.scaleRange(robotConstants.wristFront, robotConstants.wristBack);
        freeTargetPosition(0);
    }

    /**
     sets the wrist target position for the wrist servo.
     robotConstants.wristFront and robotConstants.wristBack have been set to 0 and 1.0 respectively
     */
    public void freeTargetPosition(double targetPos){
        wristPos = targetPos;
    }

    public void presetTargetPosition(wristPos wristPos){
        this.wristPos = wristPos.getPosition();
    }


    public void update(){
        wrist.setPosition(wristPos);
    }

}
