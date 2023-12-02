package org.firstinspires.ftc.teamcode.robotContainer.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TimedTrigger extends Trigger {

    double m_time;
    double m_triggerTime;
    Telemetry m_telemetry;

    public TimedTrigger(double startTime, double triggerTime, Telemetry telemetry)
    {
        m_time = startTime;
        m_triggerTime = triggerTime;
        m_telemetry = telemetry;
    }

    public void setTime(double time)
    {
        m_time = time;
    }

    @Override
    public boolean get()
    {
        m_telemetry.addData("timer time: ", (int)m_time);
        m_telemetry.update();
        if(m_time >= m_triggerTime)
        {
            m_telemetry.addData("TRIGGER ACTIVE: ", m_time);
            m_telemetry.update();
            return true;
        }
        else
        {
            return false;
        }
    }
}
