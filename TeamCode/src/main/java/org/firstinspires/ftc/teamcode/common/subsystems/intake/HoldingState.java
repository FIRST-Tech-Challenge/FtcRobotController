package org.firstinspires.ftc.teamcode.common.subsystems.intake;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/**
 * Baseline holding state that stops the intake output.
 *
 * <p>A future mechanism may use a low holding power after its physical requirements are defined.</p>
 */
public class HoldingState implements State {
    private final IntakeSubsystem intakeSubsystem;

    public HoldingState(IntakeSubsystem intakeSubsystem) {
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
        return "Holding";
    }
}
