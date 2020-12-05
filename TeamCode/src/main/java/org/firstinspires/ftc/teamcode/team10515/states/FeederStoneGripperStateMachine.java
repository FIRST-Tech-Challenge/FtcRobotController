package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class FeederStoneGripperStateMachine extends TimedState<FeederStoneGripperStateMachine.State> {
    public FeederStoneGripperStateMachine() {
        super(State.NO_GRIP);
    }

    @Override
    public String getName() {
        return "Feeder Stone Gripper State Machine";
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(0.5d, TimeUnits.SECONDS);
    }

    public enum State implements Namable {
        NO_GRIP("No Grip", 0.8d), GRIP("Grip", 0d);

        private final String name;
        private final double position;

        State(final String name, final double position) {
            this.name = name;
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

        @Override
        public String getName() {
            return name;
        }
    }
}
