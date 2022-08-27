package org.firstinspires.ftc.teamcode.core.thread.old.types.impl;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.core.thread.old.types.api.RunListenerOnceEvent;
import org.jetbrains.annotations.Contract;

/**
 * An event that does something after a certain amount of time.
 */
public class TimedEvent extends RunListenerOnceEvent {
    public final long runTime;

    /**
     * Creates a timed event to run after milliseconds.
     *
     * @param listener What will be executed after a certain amount of time.
     * @param runInMillis the amount of milliseconds it will run after.
     */
    public TimedEvent(Runnable listener, long runInMillis) {
        super(listener);
        this.runTime = System.currentTimeMillis() + runInMillis;
    }

    /**
     * Creates a timed event to run after x seconds.
     *
     * @param listener What will be executed after a certain amount of time.
     * @param runInSeconds the amount of seconds it will run after.
     */
    @NonNull
    @Contract("_, _ -> new")
    public static TimedEvent createEventWithSeconds(Runnable listener, long runInSeconds) {
        return new TimedEvent(listener, runInSeconds * 1000);
    }

    @Override
    public boolean shouldRun() {
        return runTime <= System.currentTimeMillis();
    }
}
