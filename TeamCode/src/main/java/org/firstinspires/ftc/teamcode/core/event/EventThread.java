package org.firstinspires.ftc.teamcode.core.event;

import java.util.Iterator;
import java.util.TreeSet;

public class EventThread extends Thread {
    private final TreeSet<Event> queue = new TreeSet<>();

    public void run() {
        while (!this.isInterrupted()) {
            if (!queue.isEmpty()) {
                try {
                    queue.wait();
                } catch (InterruptedException e) {
                    break;
                }
            }

            long currentTime = Event.time.nanoseconds();

            Iterator<Event> iterator =  queue.iterator();

            long waitUntil = 0;

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

    public void addEvent(Event event) {
        queue.add(event);
    }

}
