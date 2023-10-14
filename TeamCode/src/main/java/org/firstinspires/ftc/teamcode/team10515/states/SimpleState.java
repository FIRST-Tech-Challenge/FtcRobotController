package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.util.Namable;

public abstract class SimpleState<E extends Enum<E> & Namable> implements IState<E> {
    private E state;
    private E desiredState;

    protected SimpleState(E state) {
        setState(state);
        setDesiredState(state);
    }

    @Override
    public void updateState(E state) {
        setDesiredState(state);
    }

    @Override
    public boolean hasReachedStateGoal() {
        return true;
    }

    @Override
    public boolean hasReachedStateGoal(E state) {
        return state.equals(getState());
    }

    @Override
    public boolean attemptingStateChange() {
        return !getState().equals(getDesiredState());
    }

    @Override
    public E getState() {
        return state;
    }

    @Override
    public E getDesiredState() {
        return desiredState;
    }

    @Override
    public void update(double dt) {
        if(attemptingStateChange()) {
            setState(getDesiredState());
        }
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {
        telemetry.addLine(getName() + ": " + getState().getName());
    }

    protected void setState(E state) {
        this.state = state;
    }

    private void setDesiredState(E state) {
        desiredState = state;
    }
}
