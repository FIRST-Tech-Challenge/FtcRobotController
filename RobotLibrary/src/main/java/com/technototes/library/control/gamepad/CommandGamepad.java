package com.technototes.library.control.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.technototes.control.gamepad.AbstractGamepad;

public class CommandGamepad extends AbstractGamepad<CommandButton, CommandAxis> {
    public CommandGamepad(Gamepad g) {
        super(g, CommandButton.class, CommandAxis.class);
    }
}
