package org.firstinspires.ftc.teamcode.org.rustlib.commandsystem;

import java.util.ArrayList;

public class SequentialCommandGroup extends CommandGroup {

    private boolean hasStarted = false;
    private int index = 0;

    private double timeout = Double.POSITIVE_INFINITY;

    public SequentialCommandGroup(Command... commands) {
        super(commands);
    }

    public SequentialCommandGroup setTimeout(double timeout) {
        this.timeout = timeout;
        return this;
    }

    @Override
    public void initialize() {
        hasStarted = false;
        index = 0;
    }

    @Override
    public void execute() {
        if (commands[index].state == State.UNSCHEDULED) {
            if (hasStarted) {
                if (index < commands.length - 1) index++;
                hasStarted = false;
            } else {
                commands[index].schedule();
                hasStarted = true;
            }
        }
        if (timeSinceInitialized() > timeout) {
            for (Command command : commands) {
                command.cancel();
            }
            cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return index == commands.length - 1 && commands[index].isFinished();
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

        public SequentialCommandGroup build() {
            return new SequentialCommandGroup(commands.toArray(new Command[]{}));
        }
    }

    public static Builder getBuilder() {
        return new Builder();
    }
}
