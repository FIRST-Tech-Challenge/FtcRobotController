package org.firstinspires.ftc.teamcode.pedroPathing.util;

/**
 * This is the Timer class. It is an elapsed time clock with millisecond precision, or at least as
 * precise as the System.currentTimeMillis() is.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
public class Timer {
    private long startTime;

    /**
     * This creates a new Timer with the start time set to its creation time.
     */
    public Timer() {
        resetTimer();
    }

    /**
     * This resets the Timer's start time to the current time using System.currentTimeMillis().
     */
    public void resetTimer() {
        startTime = System.currentTimeMillis();
    }

    /**
     * This returns the elapsed time in milliseconds since the start time of the Timer.
     *
     * @return this returns the elapsed time in milliseconds.
     */
    public long getElapsedTime() {
        return System.currentTimeMillis() - startTime;
    }

    /**
     * This returns the elapsed time in seconds since the start time of the Timer.
     *
     * @return this returns the elapsed time in seconds.
     */
    public double getElapsedTimeSeconds() {
        return (getElapsedTime() / 1000.0);
    }
}
