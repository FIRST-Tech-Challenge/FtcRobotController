package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.util.Conversions.between;

public class RateController {

    private double m_rate;                 // factor for rate control
    private double m_maximumOutput = 1.0;       // |maximum output|
    private double m_minimumOutput = -1.0;      // |minimum output|
    private boolean m_enabled = false;                  //is the pid controller enabled
    private double m_result = 0.0;
    private long m_prevNanos; //time of previous calculate() in nanoseconds from the current epoch
    private double m_deltaTime; // time between calls to calculate() in fractional seconds
    private long m_currentNanos;

    /**
     * Allocate a RateController with the given RatePerSecond
     * Implementation assumes the emit() method will be colled
     * somewhat regularly whenever enabled
     * @param RatePerSecond
     */
    public RateController(double RatePerSecond) {

        m_rate = RatePerSecond;
        m_prevNanos = System.nanoTime();
    }

    //simplified call to set the RatePerSecond and enable in the same call to get()
    //can make the usage more readable
    public double get(double RatePerSecond){
        setRate(RatePerSecond);
        enable();
        return get();
    }

    /**
     * Emit the rate since the last call to get()
     */
    public double get() {

        // If enabled then proceed into controller calculations
        if (m_enabled) {

            //time since last iteration
            m_currentNanos = System.nanoTime();
            m_deltaTime=(m_currentNanos - m_prevNanos)/1E9;
            m_prevNanos = m_currentNanos;

            //factor the rate for deltaTime
            m_result = m_rate * m_deltaTime;

            // Make sure the final result is within bounds
            if (m_result > m_maximumOutput) {
                m_result = m_maximumOutput;
            } else if (m_result < m_minimumOutput) {
                m_result = m_minimumOutput;
            }
        }
        else m_result = 0;
        return m_result;
    }

    public void setRate(double ratePerSecond) {
        m_rate = ratePerSecond;
    }

    public synchronized double getRate() {
        return m_rate;
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output
     * @param maximumOutput the maximum value to write to the output
     */
    public void setOutputRange(double minimumOutput, double maximumOutput) {
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
    }

    public synchronized double getDeltaTime() {
        return m_deltaTime;
    }

    public void enable() {

        if (!m_enabled) m_prevNanos = System.nanoTime(); //if it's been disabled the previous time is likely very stale
        m_enabled = true;
    }

    /**
     * Stop running the Rate Controller, this forces the output to zero.
     */
    public void disable() {

        m_enabled = false;
    }

}