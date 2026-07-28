package org.firstinspires.ftc.teamcode.common.subsystems.vision;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/** First positive observation state; it advances to Tracking on the next enabled update. */
public class TargetAcquiredState implements State {
    private final VisionSubsystem visionSubsystem;

    public TargetAcquiredState(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void enter() {
        // This state makes the first positive observation visible for one loop.
    }

    @Override
    public void update() {
        visionSubsystem.updateVisionHardware();
    }

    @Override
    public void exit() {
        // Tracking or disabled behavior takes over next.
    }

    @Override
    public String getName() {
        return "TargetAcquired";
    }
}
