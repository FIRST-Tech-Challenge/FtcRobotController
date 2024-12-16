package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

public class SleeperCommand extends SounderBotCommandBase {

    private final long timeToSleepMs;

    private boolean slept = false;


    public SleeperCommand(long timeToSleepMs) {
        this.timeToSleepMs = timeToSleepMs;
    }

    @Override
    protected void doExecute() {
        if (!slept) {
            sleep(timeToSleepMs);
            slept = true;
        }
        finished = true;
    }

    @Override
    protected boolean isTargetReached() {
        // does not matter
        return false;
    }
}
