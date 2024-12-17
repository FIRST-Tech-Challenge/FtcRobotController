package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpecimenTest {
    private Servo specimenArmServo;

    private Servo specimenClawServo;
    private Telemetry telemetry;
    public SpecimenTest(Servo arm, Servo claw, Telemetry tele){
        this.specimenArmServo = arm;
        this.specimenClawServo = claw;
        this.telemetry = tele;

    }

    public void openClaw() {
        telemetry.addData("openClaw Called", "Running");
        telemetry.update();
        this.specimenClawServo.setPosition(1);
    }

    public void closeClaw() {
        telemetry.addData("closeClaw Called", "Running");
        telemetry.update();
        this.specimenClawServo.setPosition(0);
    }

    public void raiseArm(){
        telemetry.addData("raiseArm Called", "Running");
        telemetry.update();
        this.specimenArmServo.setPosition((1));
    }

    public void dropArm(){
        telemetry.addData("dropArm Called", "Running");
        telemetry.update();
        this.specimenArmServo.setPosition((0));

    }
    public void pickSpecimen() {
        telemetry.addData("pickSpecimen Called", "Running");
        telemetry.update();
        openClaw();
        this.specimenArmServo.setPosition((0.35));
    }

    public void dropSpecimen(){
        telemetry.addData("pickSpecimen Called", "Running");
        telemetry.update();
        this.specimenArmServo.setPosition((0.675));


    }



}
