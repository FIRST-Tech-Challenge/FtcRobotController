package org.firstinspires.ftc.teamcode.common.subsystems.intake;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/** Intake state that applies the configured reverse eject power. */
public class EjectingState implements State {
    private final IntakeSubsystem intakeSubsystem;

    public EjectingState(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void enter() {
        // Output is applied during update.
    }

    @Override
    public void update() {
        intakeSubsystem.runIntakeReverse();
    }

    @Override
    public void exit() {
        // The next active state controls the output.
    }

    @Override
    public String getName() {
        return "Ejecting";
    }
}
