package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Intake {
    RobotConfig r;

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

    public Intake(RobotConfig r) {
        this.r = r;
        intake = r.hardwareMap.get(Servo.class, ConfigNames.intake);
        intake.scaleRange(RobotConstants.intakeOpen, RobotConstants.intakeClosed);
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
