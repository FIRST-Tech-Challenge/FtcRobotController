package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.TelemetryWritable;
import org.firstinspires.ftc.teamcode.lib.util.Updatable;

public interface IState<E extends Enum<E> & Namable> extends TelemetryWritable, Updatable, Namable {
    void updateState(E state);
    boolean hasReachedStateGoal();
    boolean hasReachedStateGoal(E state);
    boolean attemptingStateChange();
    E getState();
    E getDesiredState();
}
