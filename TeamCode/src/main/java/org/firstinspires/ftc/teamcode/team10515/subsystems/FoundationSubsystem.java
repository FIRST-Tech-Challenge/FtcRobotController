package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.FoundationStateMachine;

public class FoundationSubsystem implements ISubsystem<FoundationStateMachine, FoundationStateMachine.State> {
    private static FoundationStateMachine foundationStateMachine;
    private RevServo leftFoundationArm;
    private RevServo rightFoundationArm;

    public FoundationSubsystem(RevServo leftFoundationArm, RevServo rightFoundationArm) {
        setFoundationStateMachine(new FoundationStateMachine());
        setLeftFoundationArm(leftFoundationArm);
        setRightFoundationArm(rightFoundationArm);
    }

    @Override
    public FoundationStateMachine getStateMachine() {
        return foundationStateMachine;
    }

    @Override
    public FoundationStateMachine.State getState() {
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
        return "Foundation Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getLeftFoundationArm().setPosition(getState().getLeftPosition());
        getRightFoundationArm().setPosition(getState().getRightPosition());
    }

    public static void setFoundationStateMachine(FoundationStateMachine foundationStateMachine) {
        FoundationSubsystem.foundationStateMachine = foundationStateMachine;
    }

    public RevServo getLeftFoundationArm() {
        return leftFoundationArm;
    }

    public void setLeftFoundationArm(RevServo leftFoundationArm) {
        this.leftFoundationArm = leftFoundationArm;
    }

    public RevServo getRightFoundationArm() {
        return rightFoundationArm;
    }

    public void setRightFoundationArm(RevServo rightFoundationArm) {
        this.rightFoundationArm = rightFoundationArm;
    }
}
