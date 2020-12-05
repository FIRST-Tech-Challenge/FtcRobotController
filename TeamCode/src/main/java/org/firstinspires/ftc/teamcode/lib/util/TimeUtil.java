package org.firstinspires.ftc.teamcode.lib.util;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.team10515.Robot;

public class TimeUtil {
    private static final double SECONDS_PER_MINUTE = 60d;
    private static final double MILLISECONDS_PER_SECOND = 1000d;
    private static final double NANOSECONDS_PER_MILLISECOND = 1000d;
    private static long         startTime;

    public static void startTime() {
        setStartTime(getAbsoluteTimeMilliseconds());
    }

    public static double getCurrentRuntime(final TimeUnits units) {
        return TimeUnits.MILLISECONDS.in(units, getAbsoluteTimeMilliseconds() - getStartTime());
    }

    public static Time getCurrentRuntime() {
        return new Time(getCurrentRuntime(TimeUnits.MILLISECONDS), TimeUnits.MILLISECONDS);
    }

    public static long getAbsoluteTimeMilliseconds() {
        return Robot.isUsingComputer() ? System.currentTimeMillis() : SystemClock.uptimeMillis();
    }

    public static double getSecondsPerMinute() {
        return SECONDS_PER_MINUTE;
    }

    public static double getMillisecondsPerSecond() {
        return MILLISECONDS_PER_SECOND;
    }

    public static double getNanosecondsPerMillisecond() {
        return NANOSECONDS_PER_MILLISECOND;
    }

    public static long getStartTime() {
        return startTime;
    }

    public static void setStartTime(long startTime) {
        TimeUtil.startTime = startTime;
    }
}
