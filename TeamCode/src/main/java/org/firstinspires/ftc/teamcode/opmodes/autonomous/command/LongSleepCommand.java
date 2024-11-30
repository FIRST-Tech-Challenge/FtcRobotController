package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

public class LongSleepCommand extends SounderBotCommandBase {
    private static final String LOG_TAG = "TimeoutTest";

    @Override
    protected void doExecute() {
        Log.i(LOG_TAG, "Sleep for 1 seconds started...");
        sleep(1000);
        Log.i(LOG_TAG, "Sleep for 1 seconds ended.");
    }

    @Override
    protected boolean isTargetReached() {
        return false;
    }
}
