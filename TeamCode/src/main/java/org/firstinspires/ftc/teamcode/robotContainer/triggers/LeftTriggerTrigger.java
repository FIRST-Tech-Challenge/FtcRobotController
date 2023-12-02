package org.firstinspires.ftc.teamcode.robotContainer.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class LeftTriggerTrigger extends Trigger {

    GamepadEx m_gamepad;
    double m_inputThreshold;

    public LeftTriggerTrigger(GamepadEx gamepad, double inputThreshold)
    {
        m_gamepad = gamepad;
        m_inputThreshold = inputThreshold;
    }

    @Override
    public boolean get()
    {
        if(m_gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > m_inputThreshold)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
