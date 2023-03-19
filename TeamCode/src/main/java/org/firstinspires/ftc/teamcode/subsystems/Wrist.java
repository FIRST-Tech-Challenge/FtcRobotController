package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Wrist extends Subsystem {

    RobotConfig r;

    public enum wristPos{
        FRONT(RobotConstants.wristFront),
        BACK(RobotConstants.wristBack),
        HALF((wristPos.FRONT.getPosition() + wristPos.BACK.getPosition())/2);

        private wristPos(double position){
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return position;
        }
    }

    Servo wrist;
    private double wristPosition;

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
        wristPosition = targetPos;
    }

    public void presetTargetPosition(wristPos wristPos){
        this.wristPosition = wristPos.getPosition();
    }


    @Override
    public void init() {
        wrist = r.opMode.hardwareMap.get(Servo.class, ConfigNames.wrist);
        presetTargetPosition(wristPos.FRONT);
        update();
    }

    @Override
    public void read() {

    }

    @Override
    public void update(){
        wrist.setPosition(wristPosition);
    }

    @Override
    public void close() {

    }

}
