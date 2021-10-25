package org.firstinspires.ftc.teamcode.core.thread.event.thread;

import org.firstinspires.ftc.teamcode.core.thread.event.types.IEvent;
import org.firstinspires.ftc.teamcode.core.thread.event.types.TimedEvent;

import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 * Thread to handle {@link TimedEvent events} as requests. Watch out, if it is overloaded with requests
 * and its not able to fulfill them faster than they are sent, it will not finish the queue.
 */
public class EventThread extends Thread {
    private final List<IEvent> queue = Collections.synchronizedList(new LinkedList<>());

    public EventThread() {
        this.setPriority(3);
    }

    public void run() {
        while (!this.isInterrupted()) {
            Iterator<IEvent> iterator = queue.iterator();

            while (iterator.hasNext()) {
                IEvent event = iterator.next();
                if (event.shouldRun()) {
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
    public void addEvent(TimedEvent event) {
        queue.add(event);
    }

}
