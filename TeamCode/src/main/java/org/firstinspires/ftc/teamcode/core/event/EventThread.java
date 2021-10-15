package org.firstinspires.ftc.teamcode.core.event;

import java.util.Iterator;
import java.util.TreeSet;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Thread to handle {@link Event events} as requests. Watch out, if it is overloaded with requests
 * and its not able to fulfill them faster than they are sent, it will not finish the queue.
 */
public class EventThread extends Thread {
    private final TreeSet<Event> queue = new TreeSet<>();
    private final ReentrantLock lock = new ReentrantLock();
    private final Object waitObject = new Object();

    public void run() {
        lock.lock();
        while (!this.isInterrupted()) {
            boolean queueEmpty;
            long fullNanos;
            long ms;
            int ns;

            queueEmpty = queue.isEmpty();

            if (!queueEmpty) {
                lock.unlock();
                try {
                    synchronized (waitObject) {
                        waitObject.wait();
                    }
                } catch (InterruptedException e) {
                    break;
                }
                lock.lock();
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
                } else {
                    waitUntil = event.runTime;
                }
            }

            fullNanos = waitUntil - currentTime;
            ms = fullNanos / 1000000;
            ns = (int) (fullNanos % 1000000);

            lock.unlock();
            try {
                queue.wait(ms, ns);
            } catch (InterruptedException e) {
                break;
            }
            lock.lock();
        }
    }

    /**
     * Adds an event to the queue.
     * @param event The event you want to add.
     * @return Whether the event was successfully added.
     */
    public boolean addEvent(Event event) {
        try {
            if (lock.tryLock(10, TimeUnit.SECONDS)) {
                queue.add(event);
                waitObject.notifyAll();
                return true;
            } else {
                return false;
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return false;
    }

}
