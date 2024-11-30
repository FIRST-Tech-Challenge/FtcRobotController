package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

public class SleeperCommand extends SounderBotCommandBase {

    private final long timeToSleepMs;


    public SleeperCommand(long timeToSleepMs) {
        this.timeToSleepMs = timeToSleepMs;
    }

    @Override
    protected void doExecute() {
        sleep(timeToSleepMs);
    }

    @Override
    protected boolean isTargetReached() {
        // does not matter
        return false;
    }
}
