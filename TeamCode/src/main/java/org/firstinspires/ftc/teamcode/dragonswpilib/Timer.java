package org.firstinspires.ftc.teamcode.dragonswpilib;

/**
 * A timer class.
 *
 * <p>Note that if the user calls SimHooks.restartTiming(), they should also reset the timer so
 * get() won't return a negative duration.
 */
public class Timer {

    private double m_startTime;
    private double m_accumulatedTime;
    private boolean m_running;

    @SuppressWarnings("MissingJavadocMethod")
    public Timer() {
        reset();
    }

    private double getMsClock() {
        return System.currentTimeMillis();
    }

    /**
     * Get the current time from the timer. If the clock is running it is derived from the current
     * system clock the start time stored in the timer class. If the clock is not running, then return
     * the time when it was last stopped.
     *
     * @return Current time value for this timer in seconds
     */
    public double get() {
        if (m_running) {
            return m_accumulatedTime + (getMsClock() - m_startTime) / 1000.0;
        } else {
            return m_accumulatedTime;
        }
    }

    /**
     * Reset the timer by setting the time to 0.
     *
     * <p>Make the timer startTime the current time so new requests will be relative now.
     */
    public void reset() {
        m_accumulatedTime = 0;
        m_startTime = getMsClock();
    }

    /**
     * Start the timer running. Just set the running flag to true indicating that all time requests
     * should be relative to the system clock. Note that this method is a no-op if the timer is
     * already running.
     */
    public void start() {
        if (!m_running) {
            m_startTime = getMsClock();
            m_running = true;
        }
    }

    /**
     * Stop the timer. This computes the time as of now and clears the running flag, causing all
     * subsequent time requests to be read from the accumulated time rather than looking at the system
     * clock.
     */
    public void stop() {
        m_accumulatedTime = get();
        m_running = false;
    }

    /**
     * Check if the period specified has passed.
     *
     * @param seconds The period to check.
     * @return Whether the period has passed.
     */
    public boolean hasElapsed(double seconds) {
        return get() >= seconds;
    }
}
