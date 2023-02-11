package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.configNames;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

public class intake {
    robotConfig r;

    public enum intakePos{
        OPEN(0),
        CLOSED(1);

        intakePos(double position){
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return position;
        }
    }

    private final Servo intake;
    private double intakePosition;

    public intake(robotConfig r) {
        this.r = r;
        intake = r.hardwareMap.get(Servo.class, configNames.intake);
        intake.scaleRange(robotConstants.intakeOpen, robotConstants.intakeClosed);
    }

    public void freeTargetPosition(double targetPos){
        this.intakePosition = targetPos;
    }

    public void presetTargetPosition(intakePos intakePos){
        this.intakePosition = intakePos.getPosition();
    }

    public void update(){
        intake.setPosition(intakePosition);
    }
}
