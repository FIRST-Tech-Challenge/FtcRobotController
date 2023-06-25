package org.firstinspires.ftc.teamcode.commandBased.commands._rr;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

public class RunCommand extends CommandBase {

    protected final Runnable m_toRun;

    /**
     * Creates a new RunCommand.  The Runnable will be run continuously until the command
     * ends.  Does not run when disabled.
     *
     * @param toRun        the Runnable to run
     * @param requirements the subsystems to require
     */
    public RunCommand(@NonNull Runnable toRun, Subsystem... requirements) {
        m_toRun = toRun;
        addRequirements(requirements);
    }

    @Override
    public void execute() {
        m_toRun.run();
    }

}
