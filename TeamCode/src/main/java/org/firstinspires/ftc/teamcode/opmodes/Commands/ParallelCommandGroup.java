package org.firstinspires.ftc.teamcode.opmodes.Commands;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class ParallelCommandGroup implements CommandGroup {
    protected ArrayList<Thread> commands;


    @Override
    public SubSystem getHardwareDevice() {
            return null;
    }

    @Override
    public void Execute() {
        for (Thread command : commands) {
            command.start();
        }

    }

    @Override
    public void addCommands(Command... commandlist) throws HardwareUsageException {
        ArrayList<SubSystem> hardware = new ArrayList<>();
        for (Command command:commandlist) {
            SubSystem device = command.getHardwareDevice();
            if (hardware.indexOf(device) ==0) {
                throw new HardwareUsageException("Hardware Object used more than once:" + device.toString());
            }
            hardware.add(command.getHardwareDevice());
            commands.add(new CommandThread(command));
        }
    }
}
