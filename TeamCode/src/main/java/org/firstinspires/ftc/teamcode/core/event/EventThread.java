package org.firstinspires.ftc.teamcode.core.event;

import java.util.Iterator;
import java.util.TreeSet;

/**
 * Thread to handle {@link Event events} as requests. Watch out, if it is overloaded with requests
 * and its not able to fulfill them faster than they are sent, it will not finish the queue.
 */
public class EventThread extends Thread {
    private final TreeSet<Event> queue = new TreeSet<>();

    public void run() {
        while (!this.isInterrupted()) {
            boolean queueEmpty;
            synchronized (queue) {
                queueEmpty = queue.isEmpty();
            }

            if (!queueEmpty) {
                try {
                    queue.wait();
                } catch (InterruptedException e) {
                    break;
                }
            }

            long currentTime = Event.time.nanoseconds();
            long waitUntil = 0;

            synchronized (queue) {
                Iterator<Event> iterator = queue.iterator();

                while (iterator.hasNext()) {
                    Event event = iterator.next();

                    if (event.runTime <= currentTime) {
                        iterator.remove();
                        // run the event
                        event.listener.run();
                    } else {
                        waitUntil = event.runTime;
                    }
                }
            }

            try {
                long fullNanos = waitUntil - currentTime;
                long ms = fullNanos / 1000000;
                int ns = (int) (fullNanos % 1000000);

                queue.wait(ms, ns);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    /**
     * Adds an event to the queue
     * @param event The event you want to add.
     */
    public void addEvent(Event event) {
        synchronized (queue) {
            queue.add(event);
            queue.notifyAll();
        }
    }

}
