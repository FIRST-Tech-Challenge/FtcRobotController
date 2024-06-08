package org.rustlib.commandsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashSet;
import java.util.Set;

public final class CommandScheduler implements Runnable {
    private static CommandScheduler instance = null;
    Set<Command> commands;
    Set<Subsystem> subsystems;
    ElapsedTime timer = new ElapsedTime();

    public CommandScheduler() {
        commands = new HashSet<>();
        subsystems = new HashSet<>();
    }

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    public void schedule(Command command) {
        command.schedule();
    }

    @Override
    public void run() {
        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }
        for (Command command : commands) {
            if (command.triggered() && command.state == Command.State.UNSCHEDULED)
                command.schedule();
        }
        for (Subsystem subsystem : subsystems) {
            subsystem.cancelConflictingCommands();
        }
        for (Command command : commands) {
            switch (command.state) {
                case QUEUED:
                    command.state = Command.State.SCHEDULED;
                    command.initializedTimestamp = command.timer.milliseconds();
                    command.initialize();
                    break;
                case SCHEDULED:
                    if (command.isFinished()) {
                        command.state = Command.State.ENDING;
                    }
                    command.execute();
                    break;
                case ENDING:
                    command.state = Command.State.UNSCHEDULED;
                    command.end(false);
                    break;
            }
        }
    }

    public void cancelAll() {
        for (Command command : commands) {
            command.cancel();
        }
    }

    public void clearRegistry() {
        subsystems = new HashSet<>();
        commands = new HashSet<>();
    }
}
