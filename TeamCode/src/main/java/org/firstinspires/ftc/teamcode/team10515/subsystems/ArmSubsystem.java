package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.ArmStateMachine;

public class ArmSubsystem implements ISubsystem<ArmStateMachine, ArmStateMachine.State> {
    public static ArmStateMachine armStateMachine;
    private RevServo armServo;

    public ArmSubsystem(RevServo dropperLeftServo){
        setArmStateMachine(new ArmStateMachine());
        setArmServo(dropperLeftServo);
    }

    @Override
    public ArmStateMachine getStateMachine() {
        return armStateMachine;
    }

    @Override
    public ArmStateMachine.State getState() {
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
        return "Arm Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getArmServo().setPosition(getState().getPosition());
    }

    public static void setArmStateMachine(ArmStateMachine dropperLeftStateMachine){
        ArmSubsystem.armStateMachine = dropperLeftStateMachine;
    }

    public RevServo getArmServo(){
        return armServo;
    }


    public void setArmServo(RevServo servo){
        this.armServo = servo;
    }

}