package org.firstinspires.ftc.teamcode.core.event;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * An event. Events are a basic way to do something after a certain amount of time.
 */
public class Event implements Comparable<Event> {
    public static final ElapsedTime time = new ElapsedTime();

    public final Runnable listener;
    public final long runTime;

    /**
     * Creates an event with nanoseconds.
     *
     * @param listener What will be executed after a certain amount of time.
     * @param runInNanos the amount of nanoseconds it will run after.
     */
    public Event(Runnable listener, long runInNanos) {
        this.listener = listener;
        this.runTime = time.nanoseconds() + runInNanos;
    }

    /**
     * Creates an event in milliseconds
     *
     * @param listener What will be executed after a certain amount of time.
     * @param runInMillis the amount of milliseconds it will run after.
     */
    public static Event createEventWithMilis(Runnable listener, long runInMillis) {
        return new Event(listener, runInMillis * 1000000);
    }

    /**
     * Creates an event in milliseconds
     *
     * @param listener What will be executed after a certain amount of time.
     * @param runInSeconds the amount of seconds it will run after.
     */
    public static Event createEventWithSeconds(Runnable listener, long runInSeconds) {
        return createEventWithMilis(listener, runInSeconds * 1000);
    }

    @Override
    public int compareTo(Event event) {
        return Long.compare(runTime, event.runTime);
    }
}
