package org.firstinspires.ftc.teamcode.utilities;

import java.util.Arrays;

public class LoopStopwatch {
    public static final int AVERAGE_OVER = 10;
    long lastTick = System.nanoTime();
    long[] ticks = new long[AVERAGE_OVER];
    private int runAvgIdx = 0;
    private int warmUpIdx = 0;
    public void clear() {
        Arrays.fill(ticks, -1);
        runAvgIdx = 0;
        warmUpIdx = 0;
        lastTick = System.nanoTime();
    }

    /**
     * Stash the current duration and return the average.
     * @return average time in seconds.
     */
    public double click() {
        long now = System.nanoTime();
        long diff = now - lastTick;
        lastTick = now;
        ticks[runAvgIdx++] = diff;
        if (warmUpIdx < AVERAGE_OVER) warmUpIdx++;
        runAvgIdx %= AVERAGE_OVER;
        return getAvg();
    }

    public double getAvg() {
        if (warmUpIdx == 0) return -1;
        long total = 0;
        for (int i = 0; i < warmUpIdx; i++) {
            total += ticks[i];
        }
        double nanos = (double) total / warmUpIdx;
        return nanos / 1e9;
    }
}
