package org.firstinspires.ftc.teamcode.common.subsystems.vision;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/** One-loop state after a tracking observation is lost before resuming the search. */
public class LostTargetState implements State {
    private final VisionSubsystem visionSubsystem;

    public LostTargetState(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void enter() {
        // This state makes the loss visible for one loop.
    }

    @Override
    public void update() {
        visionSubsystem.updateVisionHardware();
    }

    @Override
    public void exit() {
        // Searching or disabled behavior takes over next.
    }

    @Override
    public String getName() {
        return "LostTarget";
    }
}
