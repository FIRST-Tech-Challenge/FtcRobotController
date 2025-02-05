package org.firstinspires.ftc.teamcode.utility.playstationcontroller.Triggers;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class RightGamepadTrigger extends Button {
    private final GamepadEx gamepad;
    private final double valueToTrigger;

    /**
     * @param thresholdToTrigger How far the trigger has to be depressed to register (0.0 - 1.0)
     * @param gamepad            The gamepad to read the trigger of
     */
    public RightGamepadTrigger(double thresholdToTrigger, @NonNull GamepadEx gamepad) {
        this.gamepad        = gamepad;
        this.valueToTrigger = thresholdToTrigger;
    }

    @Override public boolean get() {
        return this.gamepad.getTrigger(RIGHT_TRIGGER) >= valueToTrigger;
    }
}
