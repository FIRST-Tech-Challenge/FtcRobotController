package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team.states.ClawStateMachine;

public class ClawSubsystem implements ISubsystem<ClawStateMachine, ClawStateMachine.State> {
    private static ClawStateMachine clawStateMachine;
    private RevServo ClawServo;


    public ClawSubsystem(RevServo clawServo){
        setClawStateMachine(new ClawStateMachine());
        setGripperServo(clawServo);
    }

    @Override
    public ClawStateMachine getStateMachine() {
        return clawStateMachine;
    }

    @Override
    public ClawStateMachine.State getState() {
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
        return "Gripper Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getClawServo().setPosition(getState().getPosition());
    }

    public static void setClawStateMachine(ClawStateMachine gripperStateMachine) {
        ClawSubsystem.clawStateMachine = gripperStateMachine;
    }

    public RevServo getClawServo() {
        return ClawServo;
    }


    public void setGripperServo(RevServo clawServo) {
        this.ClawServo = clawServo;
    }
}