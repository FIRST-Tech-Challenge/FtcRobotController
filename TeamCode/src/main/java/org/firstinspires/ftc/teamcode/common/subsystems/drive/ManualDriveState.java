package org.firstinspires.ftc.teamcode.common.subsystems.drive;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/** Driver-controlled mecanum drive state. */
public class ManualDriveState implements State {
    private final DriveSubsystem driveSubsystem;

    public ManualDriveState(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void enter() {
        // Motor powers are applied during update.
    }

    @Override
    public void update() {
        driveSubsystem.applyRequestedMecanumDrive();
    }

    @Override
    public void exit() {
        // The next active state controls the drive outputs.
    }

    @Override
    public String getName() {
        return "ManualDrive";
    }
}
