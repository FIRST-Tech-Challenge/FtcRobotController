package org.nknsd.robotics.team.helperClasses;

import java.util.concurrent.Callable;

public class EventPair {
    public final Callable<Boolean> listener;
    public final Runnable event;
    public final String name;

    public EventPair(Callable<Boolean> listener, Runnable event, String name) {
        this.listener = listener;
        this.event = event;
        this.name = name;
    }

    public boolean isEqualTo(Callable<Boolean> listener, Runnable event) {
        return this.listener == listener && this.event == event;
    }
}
