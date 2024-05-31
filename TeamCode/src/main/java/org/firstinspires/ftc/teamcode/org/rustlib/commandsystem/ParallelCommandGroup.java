package org.firstinspires.ftc.teamcode.org.rustlib.commandsystem;

import java.util.ArrayList;

public class ParallelCommandGroup extends CommandGroup {

    private double timeout = Double.POSITIVE_INFINITY;

    public ParallelCommandGroup(Command... commands) {
        super(commands);
    }

    public ParallelCommandGroup setTimeout(double timeout) {
        this.timeout = timeout;
        return this;
    }

    @Override
    public void initialize() {
        for (Command command : commands) {
            command.schedule();
        }
    }

    @Override
    public void execute() {
        if (timeSinceInitialized() > timeout) {
            for (Command command : commands) {
                command.cancel();
            }
            cancel();
        }
    }

    @Override
    public boolean isFinished() {
        for (Command command : commands) {
            if (command.state != State.UNSCHEDULED) {
                return false;
            }
        }
        return true;
    }

    public static class Builder {

        ArrayList<Command> commands = new ArrayList<>();

        private Builder() {
        }

        public Builder add(Command command) {
            commands.add(command);
            return this;
        }

        public Builder add(Runnable instantCommand) {
            add(new InstantCommand(instantCommand));
            return this;
        }

        public ParallelCommandGroup build() {
            return new ParallelCommandGroup(commands.toArray(new Command[]{}));
        }
    }

    public static Builder getBuilder() {
        return new Builder();
    }
}
