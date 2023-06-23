package org.firstinspires.ftc.teamcode.commandBased.classes.util;

import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * A {@link Button} that gets its state from a {@link GamepadEx}.
 *
 * @author Jackson
 */
public class GamepadTrigger extends Trigger {

    private final GamepadEx m_gamepad;
    private final GamepadKeys.Trigger[] m_triggers;


    public GamepadTrigger(GamepadEx gamepad, @NonNull GamepadKeys.Trigger... triggers) {
        m_gamepad = gamepad;
        m_triggers = triggers;
    }

    @Override
    public boolean get() {
        boolean res = true;
        for (GamepadKeys.Trigger trigger : m_triggers)
            res = res && m_gamepad.getTrigger(trigger) > 0.5;
        return res;
    }

}
