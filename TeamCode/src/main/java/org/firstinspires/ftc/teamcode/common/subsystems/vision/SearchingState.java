package org.firstinspires.ftc.teamcode.common.subsystems.vision;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/** Vision state that updates the hardware lifecycle while waiting for an external observation. */
public class SearchingState implements State {
    private final VisionSubsystem visionSubsystem;

    public SearchingState(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void enter() {
        // The hardware lifecycle is updated during each active loop.
    }

    @Override
    public void update() {
        visionSubsystem.updateVisionHardware();
    }

    @Override
    public void exit() {
        // The next state defines the next lifecycle update.
    }

    @Override
    public String getName() {
        return "Searching";
    }
}
