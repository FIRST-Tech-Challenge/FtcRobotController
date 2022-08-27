package org.firstinspires.ftc.teamcode.core.thread.old.types.api;

/**
 * An event.
 */
public interface Event {
    /**
     * Whether the event should run.
     */
    boolean shouldRun();

    /**
     * The code for the event to run.
     */
    boolean run();

    /**
     * Whether the event is cancelled.
     */
    boolean cancelled();

    /**
     * Cancels the event.
     */
    void cancel();
}
