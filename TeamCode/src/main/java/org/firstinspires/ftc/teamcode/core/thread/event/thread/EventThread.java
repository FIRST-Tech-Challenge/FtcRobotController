package org.firstinspires.ftc.teamcode.core.thread.event.thread;

import org.firstinspires.ftc.teamcode.core.thread.event.types.api.Event;

import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 * Thread to handle {@link Event events} as requests. Watch out, if it is overloaded with requests
 * and its not able to fulfill them faster than they are sent, it will not finish the queue.
 */
public class EventThread extends Thread {
    private final List<Event> queue = Collections.synchronizedList(new LinkedList<>());

    public EventThread() {
        this.setPriority(3);
    }

    public void run() {
        while (!this.isInterrupted()) {
            Iterator<Event> iterator = queue.iterator();

            while (iterator.hasNext()) {
                Event event = iterator.next();
                if (event.cancelled()) {
                    iterator.remove();
                } else if (event.shouldRun()) {
                    // run the event
                    if(event.run()) {
                        iterator.remove();
                    }
                }
            }
        }
    }

    /**
     * Adds an event to the queue.
     * @param event The event you want to add.
     */
    public void addEvent(Event event) {
        queue.add(event);
    }

}
