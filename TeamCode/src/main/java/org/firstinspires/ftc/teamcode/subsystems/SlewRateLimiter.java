package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.MathUtils;

public class SlewRateLimiter {
    private double m_positiveRateLimit;
    private double m_negativeRateLimit;
    private double m_prevVal;
    private double m_prevTime;
    private ElapsedTime timer;
    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
        m_prevVal = initialValue;
        timer = new ElapsedTime();
        m_prevTime = timer.seconds();
    }
    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }
    public double calculate(double input) {
        double currentTime = timer.seconds();
        double elapsedTime = currentTime - m_prevTime;
        m_prevVal +=
                Math.max(Math.min(input-m_prevVal, m_positiveRateLimit * elapsedTime), m_negativeRateLimit*elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }
    public void updateRateLimit(double rateLimit) {
        m_negativeRateLimit = -rateLimit;
        m_positiveRateLimit = rateLimit;
    }
    public double lastValue() {
        return m_prevVal;
    }
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = timer.seconds();
    }
}
