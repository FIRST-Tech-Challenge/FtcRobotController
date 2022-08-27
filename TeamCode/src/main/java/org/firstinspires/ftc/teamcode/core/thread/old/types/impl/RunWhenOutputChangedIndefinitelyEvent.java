package org.firstinspires.ftc.teamcode.core.thread.old.types.impl;

import java.util.function.Supplier;

import androidx.annotation.NonNull;

public class RunWhenOutputChangedIndefinitelyEvent extends RunWhenOutputChangedOnceEvent {
    private final Runnable listener;
    /**
     * @param listener The listener to be run.
     * @param output
     */
    public RunWhenOutputChangedIndefinitelyEvent(Runnable listener, @NonNull Supplier<Object> output) {
        super(listener, output);
        this.listener = listener;
    }

    @Override
    public boolean run() {
        listener.run();
        return false;
    }
}
