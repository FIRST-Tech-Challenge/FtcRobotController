package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

public class SleeperCommand extends SounderBotCommandBase {

    public SleeperCommand(long timeToSleepMs) {
        super(timeToSleepMs);
    }


    @Override
    protected void doExecute() {

    }

    @Override
    protected boolean isTargetReached() {
        // does not matter
        return false;
    }
}
