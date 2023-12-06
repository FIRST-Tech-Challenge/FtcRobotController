package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team.states.DroneStateMachine;

public class DroneSubsystem implements ISubsystem<DroneStateMachine, DroneStateMachine.State> {
    private static DroneStateMachine clawStateMachine;
    private RevServo ClawServo;


    public DroneSubsystem(RevServo clawServo){
        setClawStateMachine(new DroneStateMachine());
        setGripperServo(clawServo);
    }

    @Override
    public DroneStateMachine getStateMachine() {
        return clawStateMachine;
    }

    @Override
    public DroneStateMachine.State getState() {
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

    public static void setClawStateMachine(DroneStateMachine gripperStateMachine) {
        DroneSubsystem.clawStateMachine = gripperStateMachine;
    }

    public RevServo getClawServo() {
        return ClawServo;
    }


    public void setGripperServo(RevServo clawServo) {
        this.ClawServo = clawServo;
    }
}