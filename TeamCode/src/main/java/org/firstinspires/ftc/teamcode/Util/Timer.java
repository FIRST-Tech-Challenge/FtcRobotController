package org.firstinspires.ftc.teamcode.Util;

/**
 * This is a Timer class intended for use in Project Overload. This class
 * has nanosecond precision, or whatever precision is given by System.nanoTime().
 *
 * @ author Asher Childress - 9161 Overlaod
 */
public class Timer {

    private long startTime;

    /**
     * Constructor for Timer.
     */
    public Timer() {reset();}

    /**
     * Resets the timer.
     */
    public void reset() {
        startTime = System.nanoTime();
    }

    /**
     * Gets the time in nanoseconds.
     * @return The time in nanoseconds
     */
    public long getTime() {
        return System.nanoTime() - startTime;
    }

    /**
     * Gets the time in seconds.
     * @return The time in seconds
     */
    public double getTimeSeconds() {
        return (getTime()/ Math.pow(10.0,9));
    }
}
