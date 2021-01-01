package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;

public class ForkliftSubsystem implements ISubsystem<ForkliftStateMachine, ForkliftStateMachine.State> {
    private static ForkliftStateMachine forkliftStateMachine;
    private RevServo forkliftServoL;
    private RevServo forkliftServoR;

    public ForkliftSubsystem(/*RevServo servo1, RevServo servo2*/){
        setForkliftStateMachine(new ForkliftStateMachine());
        setServo(forkliftServoL, forkliftServoR);
    }

    @Override
    public ForkliftStateMachine getStateMachine() {
        return forkliftStateMachine;
    }

    @Override
    public ForkliftStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public String getName() {
        return "Forklift Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getServoL().setPosition(getState().getPosition());
        getServoR().setPosition(getState().getPosition());
    }

    public RevServo getServoL(){
        return forkliftServoL;
    }

    public static void setForkliftStateMachine(ForkliftStateMachine forkliftStateMachine){
        ForkliftSubsystem.forkliftStateMachine = forkliftStateMachine;
    }

    public RevServo getServoR(){
        return forkliftServoR;
    }

    public void setServo(RevServo servoL, RevServo servoR){
        this.forkliftServoL = servoL;
        this.forkliftServoR = servoR;
    }
}
