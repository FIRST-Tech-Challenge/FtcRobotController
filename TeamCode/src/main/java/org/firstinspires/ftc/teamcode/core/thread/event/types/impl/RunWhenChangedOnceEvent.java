package org.firstinspires.ftc.teamcode.core.thread.event.types.impl;

import org.firstinspires.ftc.teamcode.core.thread.event.types.api.RunListenerOnceEvent;

import java.util.concurrent.atomic.AtomicReference;

public class RunWhenChangedOnceEvent<T> extends RunListenerOnceEvent {
    private final AtomicReference<T> atomicReference;
    private long oldHash;

    /**
     * @param listener The listener to be run.
     */
    public RunWhenChangedOnceEvent(Runnable listener, AtomicReference<T> atomicReference) {
        super(listener);
        this.atomicReference = atomicReference;
        oldHash = atomicReference.get().hashCode();
    }

    @Override
    public boolean shouldRun() {
        long hash = atomicReference.get().hashCode();
        boolean changed = hash != oldHash;
        this.oldHash = hash;
        return changed;
    }
}
