package org.firstinspires.ftc.teamcode.commandBased.classes.util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandBased.classes.util.GamepadTrigger;

import java.util.HashMap;

public class TriggerGamepadEx {
    public Gamepad gamepad;

    private final HashMap<GamepadKeys.Trigger, TriggerReader> triggerReaders;
    private final HashMap<GamepadKeys.Trigger, GamepadTrigger> gamepadTriggers;

    private final GamepadKeys.Trigger[] triggers = {
            GamepadKeys.Trigger.LEFT_TRIGGER, GamepadKeys.Trigger.RIGHT_TRIGGER
    };

    public TriggerGamepadEx(Gamepad gamepad, GamepadEx ex) {
        this.gamepad = gamepad;
        triggerReaders = new HashMap<>();
        gamepadTriggers = new HashMap<>();
        for (GamepadKeys.Trigger trigger : triggers) {
            triggerReaders.put(trigger, new TriggerReader(ex, trigger));
            gamepadTriggers.put(trigger, new GamepadTrigger(ex, trigger));
        }
    }

    public double getTrigger(GamepadKeys.Trigger trigger) {
        double triggerValue = 0;
        switch (trigger) {
            case LEFT_TRIGGER:
                triggerValue = gamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                triggerValue = gamepad.right_trigger;
                break;
            default:
                break;
        }
        return triggerValue;
    }

    public boolean wasJustPressed(GamepadKeys.Trigger trigger) {
        return triggerReaders.get(trigger).wasJustPressed();
    }


    public boolean wasJustReleased(GamepadKeys.Trigger trigger) {
        return triggerReaders.get(trigger).wasJustReleased();
    }


    public void readTriggers() {
        for (GamepadKeys.Trigger trigger : triggers) {
            triggerReaders.get(trigger).readValue();
        }
    }


    public boolean isDown(GamepadKeys.Trigger trigger) {
        return triggerReaders.get(trigger).isDown();
    }


    public boolean stateJustChanged(GamepadKeys.Trigger trigger) {
        return triggerReaders.get(trigger).stateJustChanged();
    }


    public GamepadTrigger getGamepadTrigger(GamepadKeys.Trigger trigger) {
        return gamepadTriggers.get(trigger);
    }
}
