package org.firstinspires.ftc.teamcode.common.subsystems.intake;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/** Intake state that applies the configured forward intake power. */
public class IntakingState implements State {
    private final IntakeSubsystem intakeSubsystem;

    public IntakingState(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void enter() {
        // Output is applied during update.
    }

    @Override
    public void update() {
        intakeSubsystem.runIntakeForward();
    }

    @Override
    public void exit() {
        // The next active state controls the output.
    }

    @Override
    public String getName() {
        return "Intaking";
    }
}
