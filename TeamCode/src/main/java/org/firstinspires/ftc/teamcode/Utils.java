package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Utils {

    public static class RampRate {
        Double m_lastValue = 0.0;
        Double m_maxRate;
        ElapsedTime m_timer;

        public RampRate(Double maxRate) {
            m_maxRate = maxRate;
            m_timer = new ElapsedTime();
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
