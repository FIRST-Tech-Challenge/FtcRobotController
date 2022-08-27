package org.firstinspires.ftc.teamcode.core.thread.old.types.impl;

import org.firstinspires.ftc.teamcode.core.thread.old.types.api.RunListenerIndefinitelyEvent;

/**
 * An event that runs every so often.
 */
public class RunEveryTimedEvent extends RunListenerIndefinitelyEvent {
    public long nextRunTime;
    public final long runEveryMilliseconds;

    /**
     * Creates a RunEveryTimedEvent with milliseconds.
     * @param listener The listener to be run.
     * @param milliseconds Time between each run of this event.
     */
    public RunEveryTimedEvent(Runnable listener, long milliseconds) {
        super(listener);
        this.nextRunTime = System.currentTimeMillis() + milliseconds;
        this.runEveryMilliseconds = milliseconds;
    }


    @Override
    public boolean shouldRun() {
        return nextRunTime - System.currentTimeMillis() < 0;
    }

    @Override
    public boolean run() {
        nextRunTime = System.currentTimeMillis() + runEveryMilliseconds;
        return super.run();
    }
}
