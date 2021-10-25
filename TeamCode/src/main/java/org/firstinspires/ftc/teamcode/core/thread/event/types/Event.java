package org.firstinspires.ftc.teamcode.core.thread.event.types;

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
}
