package org.firstinspires.ftc.teamcode.core.thread.old.types.impl;

import org.firstinspires.ftc.teamcode.core.thread.old.types.api.RunListenerOnceEvent;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Event that checks a boolean to determine whether it should run. MUST BE USED IN ATOMIC FORM WITH
 * .SET OUTSIDE OF THE FUNCTION TO WORK.
 */
public class BooleanRunOnceEvent extends RunListenerOnceEvent {
    private final AtomicBoolean atomicBoolean;

    /**
     * @param atomicBoolean The boolean to listen to
     * @param listener The listener to be run.
     */
    public BooleanRunOnceEvent(AtomicBoolean atomicBoolean, Runnable listener) {
        super(listener);
        this.atomicBoolean = atomicBoolean;
    }

    @Override
    public boolean shouldRun() {
        return atomicBoolean.get();
    }
}
