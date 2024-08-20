package org.rustlib.rustboard;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Time {
    private static long offset;
    private long time;
    private boolean locked;
    private static final List<Time> timeRegistry = Collections.synchronizedList(new ArrayList<>());

    public static long getUTCTime() {
        return System.currentTimeMillis() + offset;
    }

    public static synchronized void calibrateUTCTime(long currentTime) {
        offset = currentTime - System.currentTimeMillis();
        timeRegistry.forEach((time) -> time.calibrate(offset));
    }

    public Time(long time, boolean locked) {
        timeRegistry.add(this);
        this.time = time;
        this.locked = locked;
    }

    public Time(long time) {
        this(time, false);
    }

    public Time() {
        this(0);
    }

    private void calibrate(double offset) {
        if (!locked) {
            time += offset;
            locked = true;
        }
    }

    public long getTimeSeconds() {
        return getTimeMS() / 1000;
    }

    public long getTimeMS() {
        return time;
    }
}
