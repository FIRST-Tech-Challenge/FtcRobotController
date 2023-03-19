package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Intake extends Subsystem {
    RobotConfig r;

    public Intake(RobotConfig r) {
        this.r = r;
    }
    public Intake(){
        r = RobotConfig.getInstance();
    }

    public enum intakePos{
        OPEN(RobotConstants.intakeOpen),
        CLOSED(RobotConstants.intakeClosed),
        INIT(RobotConstants.intakeOpen);

        intakePos(double position){
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return position;
        }
    }

    private Servo intake;
    private double intakePosition;

    public void freeTargetPosition(double targetPos){
        this.intakePosition = targetPos;
    }

    public void presetTargetPosition(intakePos intakePos){
        this.intakePosition = intakePos.getPosition();
    }

    @Override
    public void init() {
        intake = r.opMode.hardwareMap.get(Servo.class, ConfigNames.intake);
        presetTargetPosition(intakePos.INIT);
        update();
    }

    @Override
    public void read() {

    }

    public void update(){
        intake.setPosition(intakePosition);
    }

    @Override
    public void close() {}
}
