package com.SCHSRobotics.HAL9001.util.misc;

import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;

/**
 * A class used for keeping track of how much time has passed.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see HALTimeUnit
 * @since 1.1.0
 */
public class Timer {

    //The starting timestamp, in nanoseconds, of the timer.
    private long startTime = 0;
    //The duration of the timer.
    private double duration = 0;

    /**
     * Gets the time passed since the timer was last started. If the timer was not started, returns the current timestamp.
     *
     * @param timeUnit The unit of time to return.
     * @return The time passed since the timer was last started.
     * @see HALTimeUnit
     */
    public double getTimePassed(HALTimeUnit timeUnit) {
        long timeNanos = System.nanoTime() - startTime;
        return HALTimeUnit.convert(timeNanos, HALTimeUnit.NANOSECONDS, timeUnit);
    }

    /**
     * Starts the timer without setting a duration.
     */
    public void start() {
        duration = 0;
        startTime = System.nanoTime();
    }

    /**
     * Starts the timer, setting a duration.
     *
     * @param duration The duration of time that the timer will wait to pass.
     * @param timeUnit The unit of time that the duration is in.
     * @see HALTimeUnit
     */
    public void start(double duration, HALTimeUnit timeUnit) {
        this.duration = HALTimeUnit.convert(duration, timeUnit, HALTimeUnit.NANOSECONDS);
        startTime = System.nanoTime();
    }

    /**
     * Starts the timer, setting a duration.
     *
     * @param duration The duration of time that the timer will wait to pass.
     * @param timeUnit The unit of time that the duration is in.
     */
    public void start(long duration, HALTimeUnit timeUnit) {
        start((double) duration, timeUnit);
    }

    /**
     * Resets the timer's start time.
     */
    public void reset() {
        startTime = System.nanoTime();
    }

    /**
     * Gets whether the time required by the duration (set during start()) has passed. If no duration is set, will return true.
     *
     * @return Whether the time required by the duration (set during start()) has passed. If no duration is set, will return true.
     */
    public boolean requiredTimeElapsed() {
        return getTimePassed(HALTimeUnit.NANOSECONDS) > duration;
    }
}
