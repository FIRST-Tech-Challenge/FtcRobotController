package org.firstinspires.ftc.teamcode.opmodes.Commands;

public interface CommandGroup extends Command {
    void addCommands(Command ... commandlist) throws HardwareUsageException;
}
