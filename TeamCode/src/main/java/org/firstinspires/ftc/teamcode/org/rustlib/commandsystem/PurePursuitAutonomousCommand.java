package org.firstinspires.ftc.teamcode.org.rustlib.commandsystem;

import org.firstinspires.ftc.teamcode.org.rustlib.drive.FollowPathCommand;
import org.firstinspires.ftc.teamcode.org.rustlib.drive.Path;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.ArrayList;
import java.util.function.Supplier;

public class PurePursuitAutonomousCommand extends Command {
    public final Pose2d startPosition;
    private final Command command;
    private boolean timerEnabled = true;
    private Drive driveSubsystem;

    public PurePursuitAutonomousCommand(Pose2d startPosition, Command command) {
        this.startPosition = startPosition;
        this.command = command;
        recursiveInspection(command, false);
    }

    private PurePursuitAutonomousCommand(Drive driveSubsystem, Pose2d startPosition, Command command) {
        this.driveSubsystem = driveSubsystem;
        this.startPosition = startPosition;
        this.command = command;
    }

    @Override
    public void initialize() {
        driveSubsystem.odometry.setPosition(startPosition);
        command.schedule();
    }

    @Override
    public void execute() {
        if (timeSinceInitialized() > 30000 && timerEnabled) {
            cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            command.cancel();
        }
    }

    public void disableTimer() {
        timerEnabled = false;
    }

    public PurePursuitAutonomousCommand mirror() {
        return new PurePursuitAutonomousCommand(driveSubsystem, startPosition.mirror(), recursiveInspection(command, true));
    }

    private Command recursiveInspection(Command toInspect, boolean mirror) {
        ArrayList<Command> commands = new ArrayList<>();
        if (toInspect instanceof CommandGroup) {
            for (Command command : ((CommandGroup) toInspect).getCommands()) {
                command = recursiveInspection(command, mirror);
                commands.add(command);
            }
            if (toInspect instanceof SequentialCommandGroup) {
                return new SequentialCommandGroup(commands.toArray(new Command[]{}));
            } else {
                return new ParallelCommandGroup(commands.toArray(new Command[]{}));
            }
        } else {
            if (mirror) {
                return mirrorContainedPath(toInspect);
            } else {
                if (toInspect instanceof FollowPathCommand) {
                    driveSubsystem = ((FollowPathCommand) toInspect).driveSubsystem;
                }
                return toInspect;
            }
        }
    }

    private Command mirrorContainedPath(Command command) {
        if (command instanceof FollowPathCommand) {
            FollowPathCommand pathCommand = (FollowPathCommand) command;
            Supplier<Path> path = () -> pathCommand.pathSupplier.get().mirror();
            command = new FollowPathCommand(path, pathCommand.driveSubsystem);
        }
        return command;
    }
}
