package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

public abstract class SounderBotCommandBase extends CommandBase {
    private static final String LOG_TAG = SounderBotCommandBase.class.getSimpleName();
    boolean finished = false;
    long TIME_OUT_MS = 1800; // 4 seconds

    long startTime = -1;

    public  SounderBotCommandBase() {
    }

    public SounderBotCommandBase(long timeOut) {
        TIME_OUT_MS = timeOut;
    }

    @Override
    public final boolean isFinished() {
        return finished || isTargetReached();
    }

    @Override
    public final void execute() {
        if (startTime < 0) {
            startTime = System.currentTimeMillis();
            doExecute();
        } else {
            if (isTimeout()) {
                if (!finished) {
                    onTimeout();
                }
            } else {
                doExecute();
            }
        }
    }

    private boolean isTimeout() {
        if (!isDebugging()) {
            long timeUsed = System.currentTimeMillis() - startTime;
            return timeUsed > TIME_OUT_MS;
        }
        return false;
    }

    protected abstract void doExecute();

    protected void onTimeout() {
        Log.w(LOG_TAG, String.format("Command (name=%s) reached timeout (timeout=%d seconds or %d ms)", getClass().getSimpleName(), TIME_OUT_MS / 1000, TIME_OUT_MS));
        Log.w(CommonConstants.DEBUG_TAG, String.format("Command (name=%s) reached timeout (timeout=%d seconds or %d ms)", getClass().getSimpleName(), TIME_OUT_MS / 1000, TIME_OUT_MS));
        finished = true;
        end(true);
    }

    protected abstract boolean isTargetReached();

    protected void sleep(long timeInMs) {
        try {
            long sleepStartTime = System.currentTimeMillis();
            while (!finished && !isTimeout()) {
                Thread.sleep(20);
                if (System.currentTimeMillis() - sleepStartTime > timeInMs) {
                    break;
                }
            }
        } catch (InterruptedException e) {
            // ok to be interrupted
            Log.i(LOG_TAG, "sleep interrupted");
        }
    }

    protected boolean isDebugging() {
        return false;
    }

    protected void onFlagEnabled(boolean flag, Runnable runnable) {
        if (flag) {
            runnable.run();
        }
    }
}
