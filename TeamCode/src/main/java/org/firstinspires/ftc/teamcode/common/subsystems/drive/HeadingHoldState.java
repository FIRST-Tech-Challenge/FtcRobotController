package org.firstinspires.ftc.teamcode.common.subsystems.drive;

import org.firstinspires.ftc.teamcode.core.fsm.State;

/**
 * Placeholder for future IMU-based heading hold.
 *
 * <p>Until a future prompt defines IMU support, this state safely falls back to the requested
 * manual mecanum drive behavior and applies no heading correction.</p>
 */
public class HeadingHoldState implements State {
    private final DriveSubsystem driveSubsystem;

    public HeadingHoldState(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void enter() {
        // No heading target or correction exists yet.
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
        return "HeadingHold";
    }
}
