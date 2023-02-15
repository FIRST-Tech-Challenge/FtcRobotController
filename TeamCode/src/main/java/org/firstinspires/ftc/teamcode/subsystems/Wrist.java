package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Wrist {

    RobotConfig r;

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

    public Wrist(RobotConfig r) {
        this.r = r;

        wrist = r.hardwareMap.get(Servo.class, ConfigNames.wrist);
        wrist.scaleRange(RobotConstants.wristFront, RobotConstants.wristBack);
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
