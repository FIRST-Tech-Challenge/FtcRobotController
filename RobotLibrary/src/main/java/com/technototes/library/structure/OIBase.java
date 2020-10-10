package com.technototes.library.structure;

import com.technototes.library.control.gamepad.CommandGamepad;
import com.technototes.library.control.gamepad.old.OldCommandGamepad;

public abstract class OIBase {
    public CommandGamepad driverGamepad, codriverGamepad;

    public OIBase(CommandGamepad g1, CommandGamepad g2) {
        driverGamepad = g1;
        codriverGamepad = g2;
    }
}
