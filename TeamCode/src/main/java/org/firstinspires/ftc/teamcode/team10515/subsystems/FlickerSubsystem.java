package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;

public class FlickerSubsystem implements ISubsystem<FlickerStateMachine, FlickerStateMachine.State> {
    private static FlickerStateMachine flickerStateMachine;
    private RevServo flickerServo;

    public FlickerSubsystem(RevServo servo){
        setFlickerStateMachine(new FlickerStateMachine());
        setServo(flickerServo);
    }

    @Override
    public FlickerStateMachine getStateMachine() {
        return flickerStateMachine;
    }

    @Override
    public FlickerStateMachine.State getState() {
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
        return "Flicker Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getServo().setPosition(getState().getPosition());
    }

    public static void setFlickerStateMachine(FlickerStateMachine flickerStateMachine) {
        FlickerSubsystem.flickerStateMachine = flickerStateMachine;
    }

    public RevServo getServo() {
        return flickerServo;
    }

    public void setServo(RevServo flickerServo) {
        this.flickerServo = flickerServo;
    }
}
