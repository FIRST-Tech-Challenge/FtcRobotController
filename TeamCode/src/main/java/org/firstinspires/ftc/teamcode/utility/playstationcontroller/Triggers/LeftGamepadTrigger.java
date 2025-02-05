package org.firstinspires.ftc.teamcode.utility.playstationcontroller.Triggers;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class LeftGamepadTrigger extends Button {
    private final GamepadEx gamepad;
    private final double    valueToTrigger;

    /**
     * @param valueToTrigger How far the trigger has to be depressed to register (0.0 - 1.0)
     * @param gamepad        The gamepad to read the trigger from
     */
    public LeftGamepadTrigger(double valueToTrigger, @NonNull GamepadEx gamepad) {
        this.gamepad        = gamepad;
        this.valueToTrigger = valueToTrigger;
    }

    @Override public boolean get() {
        return this.gamepad.getTrigger(LEFT_TRIGGER) >= this.valueToTrigger;
    }
}
