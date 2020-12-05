package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class EndGameExtensionStateMachine extends SimpleState<EndGameExtensionStateMachine.State> {
    public EndGameExtensionStateMachine() {
        super(State.HOLD_SLIDES);
    }

    @Override
    public String getName() {
        return "End Game Extension State Machine";
    }

    public enum State implements Namable {
        HOLD_SLIDES("Hold Slides", 0.975d), RELEASE_SLIDES("Release Slides", 0.8d);

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
