package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class FlywheelStateMachine extends SimpleState<FlywheelStateMachine.State> {
    public FlywheelStateMachine() {
        super(State.IDLE);
    }

    @Override
    public String getName() {
        return "Flywheel State Machine";
    }

    public enum State implements Namable {
        IDLE("Idle", 0d),
        INTAKE("Intake", 0.6d),
        OUTTAKE("Outtake", -0.7d);

        private final String name;
        private final double power;

        State(final String name, final double power) {
            this.name  = name;
            this.power = power;
        }

        @Override
        public String getName() {
            return name;
        }

        public double getPower() {
            return power;
        }
    }
}
