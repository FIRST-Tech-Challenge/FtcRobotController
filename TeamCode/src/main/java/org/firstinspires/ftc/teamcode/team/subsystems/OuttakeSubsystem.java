package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team.states.OuttakeStateMachine;

public class OuttakeSubsystem implements ISubsystem<OuttakeStateMachine, OuttakeStateMachine.State> {
    public static OuttakeStateMachine outtakeStateMachine;
    private RevServo outtakeServo;

    public OuttakeSubsystem(RevServo outtakeServo){
        setOuttakeStateMachine(new OuttakeStateMachine());
        setOuttakeServo(outtakeServo);
    }

    @Override
    public OuttakeStateMachine getStateMachine() {
        return outtakeStateMachine;
    }

    @Override
    public OuttakeStateMachine.State getState() {
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
        return "Outtake Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getOuttakeServo().setPosition(getState().getPosition());
    }

    public static void setOuttakeStateMachine(OuttakeStateMachine outtakeStateMachineStateMachine){
        OuttakeSubsystem.outtakeStateMachine = outtakeStateMachine;
    }

    public RevServo getOuttakeServo(){
        return outtakeServo;
    }


    public void setOuttakeServo(RevServo servo){
        this.outtakeServo = servo;
    }

}