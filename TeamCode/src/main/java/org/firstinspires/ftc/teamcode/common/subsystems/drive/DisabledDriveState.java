package org.firstinspires.ftc.teamcode.common.subsystems.drive;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/** Safe drive state that continuously commands zero power. */
public class DisabledDriveState implements State {
    private final DriveSubsystem driveSubsystem;

    public DisabledDriveState(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void enter() {
        driveSubsystem.stopDrive();
    }

    @Override
    public void update() {
        driveSubsystem.stopDrive();
    }

    @Override
    public void exit() {
        // The next active state applies its own motor command.
    }

    @Override
    public String getName() {
        return "DisabledDrive";
    }
}
