
package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Commands.Command;

public abstract class Subsystem {
    private String name;
    private Command defaultCommand;

    public Subsystem(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
    }

    public Command getDefaultCommand() {
        return defaultCommand;
    }

    public void runDefaultCommand() {
        if (defaultCommand != null) {
            defaultCommand.execute();
        }
    }

    public void periodic(){};
}
