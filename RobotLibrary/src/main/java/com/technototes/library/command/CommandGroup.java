package com.technototes.library.command;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;


public abstract class CommandGroup extends Command {
    protected List<Command> commands = new LinkedList<>();

    public CommandGroup(Command... command) {
        commands.addAll(Arrays.asList(command));
    }

    public Command addCommand(Command command) {
        commands.add(command);
        return command;
    }

    @Override
    public void run() {
        switch (commandState.state) {
            case EXECUTED:
                commandState.state = State.RESET;
                return;
            default:
                runCommands();
                commandState.state = isFinished() ? State.EXECUTED : State.INITIALIZED;
        }
    }

    public abstract void runCommands();

    @Override
    public abstract boolean isFinished();
}
