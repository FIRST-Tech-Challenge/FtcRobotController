package org.firstinspires.ftc.teamcode.core.event;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Event implements Comparable<Event> {
    public static final ElapsedTime time = new ElapsedTime();

    public final Runnable listener;
    public final String name;
    public final long runTime;

    public Event(Runnable listener, String name, long runInNanos) {
        this.listener = listener;
        this.name = name;

        this.runTime = time.nanoseconds() + runInNanos;
    }

    public static Event createEventWithMilis(Runnable listener, String name, long inMilis) {
        return new Event(listener, name, inMilis * 1000000);
    }

    public static Event createEventWithSeconds(Runnable listener, String name, long inSeconds) {
        return createEventWithMilis(listener, name, inSeconds * 1000);
    }

    @Override
    public int compareTo(Event event) {
        return Long.compare(runTime, event.runTime);
    }
}
