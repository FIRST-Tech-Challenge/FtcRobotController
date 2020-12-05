package org.firstinspires.ftc.teamcode.lib.util;

public class TimeProfiler {
    private long lastUpdateTime;
    private long deltaTime;

    public TimeProfiler(final boolean runOnInitialization) {
        if(runOnInitialization) {
            start();
        }
    }

    public void start() {
        setLastUpdateTime(TimeUtil.getAbsoluteTimeMilliseconds());
    }

    public void update(final boolean reset) {
        final long updateTime = TimeUtil.getAbsoluteTimeMilliseconds();
        setDeltaTime(updateTime - getLastUpdateTime());
        if(reset) {
            setLastUpdateTime(updateTime);
        }
    }

    public double getDeltaTime(TimeUnits units) {
        return TimeUnits.MILLISECONDS.in(units, getDeltaTime());
    }

    public double getDeltaTime(TimeUnits units, boolean reset) {
        return TimeUnits.MILLISECONDS.in(units, getDeltaTime(reset));
    }

    public long getLastUpdateTime() {
        return lastUpdateTime;
    }

    public void setLastUpdateTime(long lastUpdateTime) {
        this.lastUpdateTime = lastUpdateTime;
    }

    public long getDeltaTime(boolean reset) {
        update(reset);
        return deltaTime;
    }

    public long getDeltaTime() {
        return getDeltaTime(false);
    }

    public void setDeltaTime(long deltaTime) {
        this.deltaTime = deltaTime;
    }
}
