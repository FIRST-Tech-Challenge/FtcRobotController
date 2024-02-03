package org.firstinspires.ftc.teamcode.team.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class IntakeStateMachine extends SimpleState<IntakeStateMachine.State> {
    public IntakeStateMachine() {
        super(State.IDLE);
    }

    @Override
    public String getName() {
        return "Intake State Machine";
    }

    public enum State implements Namable {
        IDLE("Idle", 0d),
        INTAKE("Intake", -0.80d),
        OUTTAKE("Outtake", 0.75d),
        DROP("Drop", 0.35d), //was .3d
        FASTDROP("fastDrop", 0.55d);

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