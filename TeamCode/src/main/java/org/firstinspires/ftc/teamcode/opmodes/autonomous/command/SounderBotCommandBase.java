package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.concurrent.atomic.AtomicBoolean;

public abstract class SounderBotCommandBase extends CommandBase {
    AtomicBoolean finished = new AtomicBoolean(false);

    @Override
    public boolean isFinished() {
        return finished.get();
    }

    protected abstract boolean isTargetReached();
}
