package org.firstinspires.ftc.teamcode.core.thread.event.api;

public abstract class AbstractEvent implements Event {
    protected boolean cancelled = false;

    @Override
    public boolean shouldRun() {
        return !cancelled;
    }

    @Override
    public boolean shouldReschedule() {
        return !cancelled;
    }


    @Override
    public void cancel() {
        cancelled = true;
    }
}
