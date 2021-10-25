package org.firstinspires.ftc.teamcode.core.thread.event.types;

public class ImmediateRunEvent extends RunListenerOnceEvent {
    public ImmediateRunEvent(Runnable listener) {
        super(listener);
    }

    @Override
    public boolean shouldRun() {
        return true;
    }
}
