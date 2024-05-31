package org.firstinspires.ftc.teamcode.org.rustlib.drive;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Subsystem;

public abstract class DriveSubsystem extends Subsystem {
    public abstract MecanumBase getBase();

    public abstract Odometry getOdometry();

    public abstract void drive(double drive, double strafe, double turn);
}
