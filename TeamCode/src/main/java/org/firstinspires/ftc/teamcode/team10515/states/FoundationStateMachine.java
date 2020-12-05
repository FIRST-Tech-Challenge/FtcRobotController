package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class FoundationStateMachine extends TimedState<FoundationStateMachine.State> {
    public FoundationStateMachine() {
        super(State.GRAB);
    }

    @Override
    public String getName() {
        return "Foundation State Machine";
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(0.5d, TimeUnits.SECONDS);
    }

    public enum State implements Namable {
        INIT("Init", 0.6d, 0.4d), GRAB("Grab", 1.0d, 0.0d);

        private final String name;
        private final double leftPosition;
        private final double rightPosition;

        State(final String name, final double leftPosition, final double rightPosition) {
            this.name          = name;
            this.leftPosition  = leftPosition;
            this.rightPosition = rightPosition;
        }

        public double getLeftPosition() {
            return leftPosition;
        }

        public double getRightPosition() {
            return rightPosition;
        }

        @Override

        public String getName() {
            return name;
        }
    }
}
