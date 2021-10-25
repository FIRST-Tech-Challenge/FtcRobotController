package org.firstinspires.ftc.teamcode.core.thread.event.types;


public class ImmediateRunEvent implements IEvent {
    private final Runnable listener;

    public ImmediateRunEvent(Runnable listener) {
        this.listener = listener;
    }

    @Override
    public boolean shouldRun() {
        return true;
    }

    @Override
    public boolean run() {
        listener.run();
        return true;
    }
}
