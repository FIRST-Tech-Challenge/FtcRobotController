package org.firstinspires.ftc.teamcode.core.event;

import java.util.Collections;
import java.util.Iterator;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * Thread to handle {@link Event events} as requests. Watch out, if it is overloaded with requests
 * and its not able to fulfill them faster than they are sent, it will not finish the queue.
 */
public class EventThread extends Thread {
    private final SortedSet<Event> queue = Collections.synchronizedSortedSet(new TreeSet<>());
    private final Object waitObject = new Object();

    public void run() {
        while (!this.isInterrupted()) {
            long fullNanos;
            long ms;
            int ns;

            synchronized (waitObject) {
                while (!queue.isEmpty()) {
                    try {
                        waitObject.wait();
                    } catch (InterruptedException e) {
                        break;
                    }
                }
            }
            long currentTime = Event.time.nanoseconds();
            long waitUntil = 0;

            Iterator<Event> iterator = queue.iterator();

            while (iterator.hasNext()) {
                Event event = iterator.next();
                if (event.runTime <= currentTime) {
                    iterator.remove();
                    // run the event
                    event.listener.run();
                } else if (waitUntil > event.runTime){
                    waitUntil = event.runTime;
                }
            }

            if(waitUntil != 0) {
                fullNanos = waitUntil - currentTime;
                ms = fullNanos / 1000000;
                ns = (int) (fullNanos % 1000000);

                synchronized (waitObject) {
                    try {
                        waitObject.wait(ms, ns);
                    } catch (InterruptedException e) {
                        break;
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
        synchronized (waitObject) {
            queue.add(event);
            waitObject.notify();
        }
    }

}
