package org.rustlib.drive;

import org.rustlib.commandsystem.Subsystem;

public abstract class DriveSubsystem extends Subsystem {
    public DriveSubsystem() {

    }

    public abstract MecanumBase getBase();

    public abstract Odometry getOdometry();

    public abstract void drive(double drive, double strafe, double turn);
}
