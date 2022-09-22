package org.firstinspires.ftc.teamcode.util;

public final class MidpointTimer {
    private final long beginTs = System.nanoTime();
    private long lastTime;

    public double seconds() {
        return 1e-9 * (System.nanoTime() - beginTs);
    }

    public double addSplit() {
        long time = System.nanoTime() - beginTs;
        double midTimeSecs = 0.5e-9 * (lastTime + time);
        lastTime = time;
        return midTimeSecs;
    }
}
