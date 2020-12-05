package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class FeederStateMachine implements IState<FeederStateMachine.State> {
    public FeederStateMachine() {

    }

    @Override
    public void updateState(State state) {

    }

    @Override
    public boolean hasReachedStateGoal() {
        return false;
    }

    @Override
    public boolean hasReachedStateGoal(State state) {
        return false;
    }

    @Override
    public boolean attemptingStateChange() {
        return false;
    }

    @Override
    public State getState() {
        return null;
    }

    @Override
    public State getDesiredState() {
        return null;
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {

    }

    public enum State implements Namable {
        ;

        @Override
        public String getName() {
            return null;
        }
    }
}
