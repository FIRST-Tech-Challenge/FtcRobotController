package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class CapstoneStateMachine extends TimedState<CapstoneStateMachine.State> {
    public CapstoneStateMachine() {
        super(State.HOLD);
    }

    @Override
    public String getName() {
        return "Capstone State Machine";
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(1d, TimeUnits.SECONDS);
    }

    public enum State implements Namable {
        HOLD(0.6d), DROP_CAPSTONE(0.0d),DROP_FOUNDATION(1.0d);

        private final double position;

        State(final double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

        @Override
        public String getName() {
            return null;
        }
    }
}
