package org.firstinspires.ftc.teamcode.core.thread.event.types.api;

import java.util.concurrent.atomic.AtomicBoolean;

public abstract class EventAbstract implements Event {
    private final AtomicBoolean atomicBoolean = new AtomicBoolean();

    @Override
    public boolean cancelled() {
        return atomicBoolean.get();
    }

    @Override
    public void cancel() {
        atomicBoolean.set(true);
    }
}
