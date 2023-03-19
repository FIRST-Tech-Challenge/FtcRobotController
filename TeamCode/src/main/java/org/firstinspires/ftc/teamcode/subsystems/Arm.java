package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Arm extends Subsystem {
    RobotConfig r;

    public enum armPos{
        FRONT(RobotConstants.armFront),
        BACK(RobotConstants.armBack),
        FRONT_DELIVERY(RobotConstants.armFrontDelivery),
        BACK_DELIVERY(RobotConstants.armBackDelivery),
        HALF(RobotConstants.armMiddle);

        armPos(double position){
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return position;
        }
    }

    Servo arm;
    private double armPosition;

    public Arm(RobotConfig r) {
        this.r = r;
    }
    public Arm(){
        r = RobotConfig.getInstance();
    }

    public void freeTargetPosition(double targetPos){
        armPosition = targetPos;
    }

    public void presetTargetPosition(armPos armPosition){
        if(r.delivery){
            this.armPosition = armPos.valueOf(armPosition.toString() + "_DELIVERY").getPosition();
        }
        else{
            this.armPosition = armPosition.getPosition();
        }
    }

    @Override
    public void init() {
        arm = r.opMode.hardwareMap.get(Servo.class, ConfigNames.arm);
        presetTargetPosition(armPos.FRONT);
        update();
    }

    @Override
    public void read() {}

    @Override
    public void update(){
        arm.setPosition(armPosition);
    }

    @Override
    public void close() {}
}
