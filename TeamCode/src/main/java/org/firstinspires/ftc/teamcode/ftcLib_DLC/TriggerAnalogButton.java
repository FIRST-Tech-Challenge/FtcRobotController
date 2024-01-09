package org.firstinspires.ftc.teamcode.ftcLib_DLC;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * converts the analog Trigger to a digital Button from the GamepadEx of FTClib
 */
public class TriggerAnalogButton extends Button {

    private double threshold;
    private GamepadEx gamepad;
    private GamepadKeys.Trigger trigger;
    /**
     * @param gamepad
     * @param trigger The enum responsible for what trigger to fetch
     * @param threshold Button is activated when the analog value passes this threshold
     */
    public TriggerAnalogButton (GamepadEx gamepad, GamepadKeys.Trigger trigger, double threshold)
    {
        this.threshold = threshold;
        this.trigger=trigger;
        this.gamepad=gamepad;
    }

    public boolean get() {
        return (gamepad.getTrigger(trigger)>threshold);
    }
}