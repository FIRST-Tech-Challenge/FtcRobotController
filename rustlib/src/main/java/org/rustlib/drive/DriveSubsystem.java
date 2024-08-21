package org.rustlib.drive;

import org.rustlib.commandsystem.Subsystem;
import org.rustlib.geometry.Rotation2d;

public abstract class DriveSubsystem extends Subsystem {
    protected Rotation2d fieldCentricOffset = new Rotation2d();

    public abstract MecanumBase getBase();

    public abstract Odometry getOdometry();

    public abstract void drive(double drive, double strafe, double turn);

    public void setFieldCentricOffset(Rotation2d fieldCentricOffset) {
        this.fieldCentricOffset = fieldCentricOffset;
    }
}
