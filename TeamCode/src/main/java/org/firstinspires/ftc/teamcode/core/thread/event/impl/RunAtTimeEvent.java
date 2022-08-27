package org.firstinspires.ftc.teamcode.core.thread.event.impl;

import org.firstinspires.ftc.teamcode.core.thread.event.api.AbstractEvent;

public class RunAtTimeEvent extends AbstractEvent {
    private final Runnable runnable;
    private final long runTime;

    /**
     * @param runnable The code to be run
     * @param runTime The time at which the runnable will run
     */
    public RunAtTimeEvent(Runnable runnable, long runTime) {
        this.runnable = runnable;
        this.runTime = runTime;
    }

    @Override
    public void run() {
        runnable.run();
    }

    @Override
    public boolean shouldReschedule() {
        return false;
    }

    @Override
    public long reschedule() {
        return 0;
    }

    @Override
    public long schedule() {
        return runTime;
    }
}
