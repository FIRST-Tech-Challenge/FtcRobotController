package org.rustlib.drive;

import org.rustlib.commandsystem.Command;

import java.util.function.Supplier;

public class FollowPathCommand extends Command {
    public final Supplier<Path> pathSupplier;
    public final DriveSubsystem driveSubsystem;

    public FollowPathCommand(Supplier<Path> pathSupplier, DriveSubsystem drive) {
        this.pathSupplier = pathSupplier;
        driveSubsystem = drive;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.getBase().setFollowPath(pathSupplier.get());
    }

    @Override
    public void execute() {
        driveSubsystem.getBase().followPath();
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.getBase().finishedFollowing();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.getBase().enableBraking();
        if (!interrupted) driveSubsystem.drive(0, 0, 0);
    }
}
