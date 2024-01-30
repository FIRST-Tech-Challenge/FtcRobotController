package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.HangStateMachine;

public class HangSubsystem implements ISubsystem<HangStateMachine, HangStateMachine.State> {
    private static HangStateMachine HangStateMachine;
    private RevMotor HangWheels;

    public HangSubsystem(RevMotor HangMotor){
        setHangStateMachine(new HangStateMachine());
        setHangWheels(HangMotor);
    }

    @Override
    public HangStateMachine getStateMachine() {
        return HangStateMachine;
    }

    @Override
    public HangStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getHangWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getHangWheels().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Hang Subsystem";
    }

    private static void setHangStateMachine(HangStateMachine HangSM) {
        HangSubsystem.HangStateMachine = HangSM;
    }

    private void setHangWheels(RevMotor HangMotor){
        this.HangWheels = HangMotor;
    }
    private RevMotor getHangWheels(){
        return HangWheels;
    }
}