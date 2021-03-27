package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class TurnCommand extends PathCommand {
    public double angle;
    public TurnCommand(DrivebaseSubsystem sub, double ang) {
        super(sub);
        angle = ang;
    }

    @Override
    public void init() {
        subsystem.turnAsync(Math.toRadians(angle)-subsystem.getPoseEstimate().getHeading());
    }

}
