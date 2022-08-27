package org.firstinspires.ftc.teamcode.core.thread.old.types.impl;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.core.thread.old.types.api.RunListenerOnceEvent;

import java.util.concurrent.atomic.AtomicReference;

public class RunWhenVariableChangedOnceEvent<T> extends RunListenerOnceEvent {
    private final AtomicReference<T> atomicReference;
    private long oldHash;

    /**
     * @param listener The listener to be run.
     */
    public RunWhenVariableChangedOnceEvent(Runnable listener, @NonNull AtomicReference<T> atomicReference) {
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
