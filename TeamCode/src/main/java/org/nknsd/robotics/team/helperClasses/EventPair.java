package org.nknsd.robotics.team.helperClasses;

import java.util.concurrent.Callable;

public class EventPair {
    public final Callable<Boolean> listener;
    public final Runnable event;

    public EventPair(Callable<Boolean> listener, Runnable event) {
        this.listener = listener;
        this.event = event;
    }

    public boolean isEqualTo(Callable<Boolean> listener, Runnable event) {
        return this.listener == listener && this.event == event;
    }
}
