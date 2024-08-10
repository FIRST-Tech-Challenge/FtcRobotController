package com.qualcomm.robotcore.util;


import android.annotation.SuppressLint;

import java.util.concurrent.TimeUnit;

/**
 * The {@link ElapsedTime} class provides a simple handy timer to measure elapsed time intervals.
 * The timer does not provide events or callbacks, as some other timers do. Rather, at an application-
 * determined juncture, one can {@link #reset()} the timer. Thereafter, one can query the interval
 * of wall-clock time that has subsequently elapsed by calling the {@link #time()}, {@link #seconds()},
 * or {@link #milliseconds()} methods. The timer has nanosecond internal accuracy. The precision
 * reported by the {@link #time()} method is either seconds or milliseconds, depending on how the
 * timer is initially constructed.
 *
 * This class is thread-safe.
 */
@SuppressLint("DefaultLocale")
@SuppressWarnings("WeakerAccess")
public class ElapsedTime {

    //------------------------------------------------------------------------------------------------
    // Types and constants
    //------------------------------------------------------------------------------------------------

    /**
     * An indicator of the resolution of a timer.
     * @see ElapsedTime#ElapsedTime(Resolution)
     */
    public enum Resolution {
        SECONDS,
        MILLISECONDS
    }

    /** the number of nanoseconds in a second */
    public static final long SECOND_IN_NANO = 1000000000;

    /** the number of nanoseconds in a millisecond */
    public static final long MILLIS_IN_NANO = 1000000;

    //------------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------------

    protected volatile long nsStartTime;
    protected final double resolution;

    //------------------------------------------------------------------------------------------------
    // Construction
    //------------------------------------------------------------------------------------------------

    /**
     * Creates a timer with resolution {@link com.qualcomm.robotcore.util.ElapsedTime.Resolution#SECONDS Resolution.Seconds}
     * that is initialized with the now-current time.
     * @see #ElapsedTime(long)
     * @see #ElapsedTime(Resolution)
     */
    public ElapsedTime() {
        reset();
        this.resolution = SECOND_IN_NANO;
    }

    /**
     * Creates a timer with resolution {@link com.qualcomm.robotcore.util.ElapsedTime.Resolution#SECONDS Resolution.Seconds}.
     * The timer is initialized with the provided start time. Zero is often a useful value to provide
     * here: in common usage such timers will often be processed by application logic virtually immediately.
     *
     * @param startTime the initial value of the timer
     * @see #ElapsedTime()
     */
    public ElapsedTime(long startTime) {
        this.nsStartTime = startTime;
        this.resolution = SECOND_IN_NANO;
    }

    /**
     * Creates a timer with a resolution of seconds or milliseconds. The resolution
     * affects the units in which the {@link #time()} method reports. The timer is initialized
     * with the current time.
     * @param resolution the resolution of the new timer
     * @see #ElapsedTime()
     */
    public ElapsedTime(Resolution resolution) {
        reset();
        switch (resolution) {
            case SECONDS:
            default:
                this.resolution = SECOND_IN_NANO;
                break;
            case MILLISECONDS:
                this.resolution = MILLIS_IN_NANO;
                break;
        }
    }

    //------------------------------------------------------------------------------------------------
    // Operations
    //------------------------------------------------------------------------------------------------

    protected long nsNow() {
        return System.nanoTime();
    }

    /**
     * Returns the current time on the clock used by the timer
     * @param unit  the time unit in which the current time should be returned
     * @return      the current time on the clock used by the timer
     */
    public long now(TimeUnit unit) {
        return unit.convert(nsNow(), TimeUnit.NANOSECONDS);
    }

    /**
     * Resets the internal state of the timer to reflect the current time. Instantaneously following
     * this reset, {@link #time()} will report as zero.
     * @see #time()
     */
    public void reset() {
        nsStartTime = nsNow();
    }

    /**
     * Returns, in resolution-dependent units, the time at which this timer was last reset.
     * @return the reset time of the timer
     */
    public double startTime() {
        return nsStartTime / resolution;
    }

    /**
     * Returns the time at which the timer was last reset, in units of nanoseconds
     * @return the time at which the timer was last reset, in units of nanoseconds
     */
    public long startTimeNanoseconds() {
        return this.nsStartTime;
    }

    /**
     * Returns the duration that has elapsed since the last reset of this timer.
     * Units used are either seconds or milliseconds, depending on the resolution with
     * which the timer was instantiated.
     * @return time duration since last timer reset
     * @see #ElapsedTime()
     * @see #ElapsedTime(Resolution)
     * @see #seconds()
     * @see #milliseconds()
     */
    public double time() {
        return (nsNow() - nsStartTime) / resolution;
    }

    /**
     * Returns the duration that has elapsed since the last reset of this timer
     * as an integer in the units requested.
     * @param unit  the units in which to return the answer
     * @return time duration since last timer reset
     */
    public long time(TimeUnit unit) {
        return unit.convert(nanoseconds(), TimeUnit.NANOSECONDS);
    }

    /**
     * Returns the duration that has elapsed since the last reset of this timer in seconds
     * @return time duration since last timer reset
     * @see #time()
     */
    public double seconds() {
        return nanoseconds() / ((double)(SECOND_IN_NANO));
    }

    /**
     * Returns the duration that has elapsed since the last reset of this timer in milliseconds
     * @return time duration since last timer reset
     * @see #time()
     */
    public double milliseconds() {
        return seconds() * 1000;
    }

    /**
     * Returns the duration that has elapsed since the last reset of this timer in nanoseconds
     * @return time duration since last timer reset
     * @see #time()
     */
    public long nanoseconds() {
        return (nsNow() - nsStartTime);
    }

    /**
     * Returns the resolution with which the timer was instantiated.
     * @return the resolution of the timer
     */
    public Resolution getResolution() {
        if (this.resolution == MILLIS_IN_NANO)
            return Resolution.MILLISECONDS;
        else
            return Resolution.SECONDS;
    }

    //------------------------------------------------------------------------------------------------
    // Utility
    //------------------------------------------------------------------------------------------------

    private String resolutionStr() {
        if (resolution == SECOND_IN_NANO) {
            return "seconds";
        } else if (resolution == MILLIS_IN_NANO) {
            return "milliseconds";
        } else {
            return "unknown units";
        }
    }

    /**
     * Log a message stating how long the timer has been running
     */
    public void log(String label) {
        // !!! RobotLog.v(String.format("TIMER: %20s - %1.3f %s", label, time(), resolutionStr()));
    }

    /**
     * Returns a string indicating the current elapsed time of the timer.
     */
    @Override
    public String toString() {
        return String.format("%1.4f %s", time(), resolutionStr());
    }
}
