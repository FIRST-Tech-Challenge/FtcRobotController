package org.firstinspires.ftc.teamcode.pedroPathing.util;

/**
 * This is the NanoTimer class. It is an elapsed time clock with nanosecond precision, or at least
 * as precise as the System.nanoTime() is.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
public class NanoTimer {
    private long startTime;

    /**
     * This creates a new NanoTimer with the start time set to its creation time.
     */
    public NanoTimer() {
        resetTimer();
    }

    /**
     * This resets the NanoTimer's start time to the current time using System.nanoTime().
     */
    public void resetTimer() {
        startTime = System.nanoTime();
    }

    /**
     * This returns the elapsed time in nanoseconds since the start time of the NanoTimer.
     *
     * @return this returns the elapsed time in nanoseconds.
     */
    public long getElapsedTime() {
        return System.nanoTime() - startTime;
    }

    /**
     * This returns the elapsed time in seconds since the start time of the NanoTimer.
     *
     * @return this returns the elapsed time in seconds.
     */
    public double getElapsedTimeSeconds() {
        return (getElapsedTime() / Math.pow(10.0,9));
    }
}
