package org.firstinspires.ftc.teamcode.core.thread;

import org.firstinspires.ftc.teamcode.core.thread.types.api.Event;

import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.BooleanSupplier;

/**
 * Thread to handle {@link Event events} as requests. Watch out, if it is overloaded with requests
 * and its not able to fulfill them faster than they are sent, it will not finish the queue.
 */
public class EventThread extends Thread {
    private final ConcurrentLinkedQueue<Event> queue = new ConcurrentLinkedQueue<>();
    private final BooleanSupplier continueRunning;

    public EventThread() {
        this(() -> true);
    }

    public EventThread(BooleanSupplier continueRunning) {
        this.setPriority(3);
        this.continueRunning = continueRunning;
    }

    public void run() {
        while (!this.isInterrupted() || continueRunning.getAsBoolean()) {
            if(queue.isEmpty()) {
                Thread.yield();
                continue;
            }
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
