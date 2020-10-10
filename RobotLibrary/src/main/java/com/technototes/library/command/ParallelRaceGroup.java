package com.technototes.library.command;

public class ParallelRaceGroup extends CommandGroup {
    public ParallelRaceGroup(Command... commands) {
        super(commands);
    }

    @Override
    public void run() {
        if (isFinished()) {
            commandState.state = State.RESET;
        } else {
            commands.forEach((command) -> run());
        }
    }

    @Override
    public boolean isFinished() {
        for (Command c : commands) {
            if (c.commandState.state == State.RESET) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void runCommands() {
        commands.forEach(command -> run());
    }
}
