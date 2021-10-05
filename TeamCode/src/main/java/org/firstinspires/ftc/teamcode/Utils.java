package org.firstinspires.ftc.teamcode;

public class Utils {

    public static class RampRate {
        Double m_lastValue = 0.0;
        Double m_maxRate;

        public RampRate(Double maxRate) {
            m_maxRate = maxRate;
        }

        public Double update(Double current) {
            Double maxValue = m_lastValue * (1 + m_maxRate);
            if (current > maxValue) {
                m_lastValue = maxValue;
                return maxValue;
            } else {
                m_lastValue = current;
                return current;
            }
        }
    }
}
