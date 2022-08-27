package org.firstinspires.ftc.teamcode.core.thread;


import org.firstinspires.ftc.teamcode.core.thread.event.api.Event;

import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

public class EventHelper implements AutoCloseable {
    private final Timer timer = new Timer();

    public void addEvent(Event event) {
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                if(event.shouldRun()) {
                    event.run();
                }
                if(event.shouldReschedule()) {
                    timer.schedule(this, event.reschedule());
                }
            }
        }, new Date(event.schedule()));
    }

    public void close() {
        timer.cancel();
    }
}
