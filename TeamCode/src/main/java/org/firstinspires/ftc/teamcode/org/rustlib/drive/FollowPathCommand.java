package org.firstinspires.ftc.teamcode.org.rustlib.drive;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.function.Supplier;

public class FollowPathCommand extends Command {
    public final Supplier<Path> pathSupplier;
    public final Drive driveSubsystem;

    public FollowPathCommand(Supplier<Path> pathSupplier, Drive drive) {
        this.pathSupplier = pathSupplier;
        driveSubsystem = drive;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.base.setFollowPath(pathSupplier.get());
    }

    @Override
    public void execute() {
        driveSubsystem.base.followPath();
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.base.finishedFollowing();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.base.enableBraking();
        if (!interrupted) driveSubsystem.drive(0, 0, 0);
    }
}
