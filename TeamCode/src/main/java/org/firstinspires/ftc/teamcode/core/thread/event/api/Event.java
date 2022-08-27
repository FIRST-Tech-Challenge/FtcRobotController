package org.firstinspires.ftc.teamcode.core.thread.event.api;

public interface Event {
    /**
     * Code that runs when the event runs.
     */
    void run();

    /**
     * @return whether the event should run
     */
    boolean shouldRun();

    /**
     * @return If the event should reschedule.
     */
    boolean shouldReschedule();

    /**
     * @return Unix Epoch (in milliseconds) that represents the next run time.
     */
    long reschedule();

    /**
     * @return Unix Epoch (in milliseconds) that represents the first run time.
     */
    long schedule();

    /**
     * Cancels the event, effectively making shouldRun and shouldReschedule false.
     */
    void cancel();
}
