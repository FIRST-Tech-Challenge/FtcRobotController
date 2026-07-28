package org.firstinspires.ftc.teamcode.common.subsystems.intake;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/** Safe intake state that continuously commands zero output. */
public class IdleIntakeState implements State {
    private final IntakeSubsystem intakeSubsystem;

    public IdleIntakeState(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void enter() {
        intakeSubsystem.stopOutput();
    }

    @Override
    public void update() {
        intakeSubsystem.stopOutput();
    }

    @Override
    public void exit() {
        // The next active state applies its own output.
    }

    @Override
    public String getName() {
        return "IdleIntake";
    }
}
