package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class VirtualFourBarStateMachine extends TimedState<VirtualFourBarStateMachine.State> {
    public VirtualFourBarStateMachine() {
        super(State.INIT);
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(1d, TimeUnits.SECONDS);
    }

    @Override
    public String getName() {
        return "Virtual Four Bar State Machine";
    }

    public enum State implements Namable {
        INIT("Init", 0d, 1d),
        IN_ROBOT("In Robot", 0d, 1d),
        STACK("Stack", 1d, 0d);

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
