package org.firstinspires.ftc.teamcode.org.rustlib.commandsystem;

public abstract class CommandGroup extends Command {
    final Command[] commands;

    public CommandGroup(Command... commands) {
        this.commands = commands;
    }

    public Command[] getCommands() {
        return commands;
    }
}
