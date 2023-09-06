package org.firstinspires.ftc.teamcode.opmodes.Commands;


import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

import java.util.Arrays;
import java.util.List;

public class SequentialCommandGroup implements CommandGroup {
    protected List<Command> commands;


    @Override
    public void Execute() {
        for (Command e : commands) {
            e.Execute();
        }

    }

    @Override
    public SubSystem getHardwareDevice() {
        return null;
    }

    @Override
    public void addCommands(Command... commands) {
        this.commands = Arrays.asList(commands);
    }
}
