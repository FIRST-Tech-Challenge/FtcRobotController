package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.IntakeStateMachine;

public class IntakeSubsystem implements ISubsystem<IntakeStateMachine, IntakeStateMachine.State> {
    private static IntakeStateMachine intakeStateMachine;
    private RevMotor intakeWheels;

    public IntakeSubsystem(RevMotor intakeMotor){
        setIntakeStateMachine(new IntakeStateMachine());
        setIntakeWheels(intakeMotor);
    }

    @Override
    public IntakeStateMachine getStateMachine() {
        return intakeStateMachine;
    }

    @Override
    public IntakeStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getIntakeWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getIntakeWheels().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Flywheel Subsystem";
    }

    private static void setIntakeStateMachine(IntakeStateMachine intakeStateMachine) {
        IntakeSubsystem.intakeStateMachine = intakeStateMachine;
    }

    private void setIntakeWheels(RevMotor intakeMotor){
        this.intakeWheels = intakeMotor;
    }
    private RevMotor getIntakeWheels(){
        return intakeWheels;
    }
}