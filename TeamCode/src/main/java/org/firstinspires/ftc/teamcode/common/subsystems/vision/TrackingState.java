package org.firstinspires.ftc.teamcode.common.subsystems.vision;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/** Vision state that remains active while external reports continue to detect a target. */
public class TrackingState implements State {
    private final VisionSubsystem visionSubsystem;

    public TrackingState(VisionSubsystem visionSubsystem) {
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
        // Lost-target or disabled behavior takes over next.
    }

    @Override
    public String getName() {
        return "Tracking";
    }
}
