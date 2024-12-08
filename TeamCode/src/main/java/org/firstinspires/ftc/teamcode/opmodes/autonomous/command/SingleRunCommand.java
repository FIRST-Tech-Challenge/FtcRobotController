package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.command.Subsystem;

public class SingleRunCommand extends SounderBotCommandBase {

    final Runnable runnable;

    boolean runFinished = false;

    public SingleRunCommand(Runnable runnable, Subsystem... subsystems) {
        this.runnable = runnable;
        addRequirements(subsystems);
    }

    @Override
    protected void doExecute() {
        try {
            runnable.run();
        } finally {
            runFinished = true;
        }
    }

    @Override
    protected boolean isTargetReached() {
        return runFinished;
    }
}
