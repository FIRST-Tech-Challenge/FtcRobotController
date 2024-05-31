package org.firstinspires.ftc.teamcode.org.rustlib.commandsystem;

import java.util.ArrayList;

public abstract class Subsystem {
    ArrayList<Command> requirements = new ArrayList<>();
    Command defaultCommand = null;

    public Subsystem() {
        CommandScheduler.getInstance().subsystems.add(this);
    }

    final void cancelConflictingCommands() {
        Command toRun = null;
        int priority;
        for (Command requirement : requirements) {
            if (requirement.state == Command.State.QUEUED || requirement.state == Command.State.SCHEDULED) {
                if (toRun == null)
                    priority = -1;
                else
                    priority = toRun.type.ordinal();

                if (requirement.type.ordinal() > priority) {
                    if (toRun != null) toRun.cancel();
                    toRun = requirement;
                } else {
                    if (requirement.type.ordinal() < priority || requirement.scheduledTimestamp < toRun.scheduledTimestamp) {
                        requirement.cancel();
                    } else {
                        toRun.cancel();
                        toRun = requirement;
                    }
                }
            }
        }
    }

    public void periodic() {

    }

    public final void setDefaultCommand(Command command) {
        if (command == null) return;
        command.type = Command.Type.DEFAULT_COMMAND;
        if (defaultCommand != null) defaultCommand.type = Command.Type.NORMAL;
        defaultCommand = command;
    }
}
