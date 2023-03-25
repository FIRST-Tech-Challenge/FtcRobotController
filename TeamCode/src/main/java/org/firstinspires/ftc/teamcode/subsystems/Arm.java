package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Arm extends Subsystem {
    RobotConfig r;

    public enum ArmPos {
        FRONT(RobotConstants.armFront),
        BACK(RobotConstants.armBack),
        FRONT_DELIVERY(RobotConstants.armFrontDelivery),
        BACK_DELIVERY(RobotConstants.armBackDelivery),
        HALF(RobotConstants.armMiddle);

        ArmPos(double position){
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return position;
        }
    }

    Servo arm;
    private double armPosition;
    private ArmPos armPositionConstant;

    public Arm(RobotConfig r) {
        this.r = r;
    }
    public Arm(){
        r = RobotConfig.getInstance();
    }

    public void freeTargetPosition(double targetPos){
        armPosition = targetPos;
    }

    public void presetTargetPosition(ArmPos armPosition){
        armPositionConstant = armPosition;
    }

    @Override
    public void init() {
        arm = r.opMode.hardwareMap.get(Servo.class, ConfigNames.arm);
        presetTargetPosition(ArmPos.FRONT);
        update();
    }

    @Override
    public void read() {}

    @Override
    public void update(){
        switch (armPositionConstant){
            case FRONT_DELIVERY:
                if(r.isDelivery()){
                    this.armPosition = ArmPos.FRONT_DELIVERY.getPosition();
                }
                else if (r.isPickup()) {
                    this.armPosition = ArmPos.FRONT.getPosition();
                }
                else {
                    this.armPosition = ArmPos.HALF.getPosition();
                }
                break;
            case FRONT:
                if (r.isPickup()) {
                    this.armPosition = ArmPos.FRONT.getPosition();
                }
                else {
                    this.armPosition = ArmPos.HALF.getPosition();
                }
                break;
            case BACK_DELIVERY:
                if(r.isDelivery()){
                    this.armPosition = ArmPos.BACK_DELIVERY.getPosition();
                }
                else if (r.isPickup()) {
                    this.armPosition = ArmPos.BACK.getPosition();
                }
                else {
                    this.armPosition = ArmPos.HALF.getPosition();
                }
                break;
            case BACK:
                if (r.isPickup()) {
                    this.armPosition = ArmPos.BACK.getPosition();
                }
                else {
                    this.armPosition = ArmPos.HALF.getPosition();
                }
                break;
            default:
                this.armPosition = ArmPos.HALF.getPosition();
                break;
        }
        arm.setPosition(armPosition);
    }

    @Override
    public void close() {}
}
