package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Wrist extends Subsystem {

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
    }
    public Wrist(){
        r = RobotConfig.getInstance();
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


    @Override
    public void init() {
        wrist = r.opMode.hardwareMap.get(Servo.class, ConfigNames.wrist);
        wrist.scaleRange(RobotConstants.wristFront, RobotConstants.wristBack);
        freeTargetPosition(0);
    }

    @Override
    public void read() {

    }

    public void update(){
        wrist.setPosition(wristPos);
    }

    @Override
    public void close() {

    }

}
