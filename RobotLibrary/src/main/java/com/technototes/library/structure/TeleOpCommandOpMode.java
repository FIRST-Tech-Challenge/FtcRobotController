package com.technototes.library.structure;

import com.technototes.library.control.gamepad.CommandGamepad;
import com.technototes.library.control.gamepad.old.OldCommandGamepad;
@Deprecated
public abstract class TeleOpCommandOpMode extends CommandOpMode {
    public CommandGamepad driverGamepad = new CommandGamepad(gamepad1);
    public CommandGamepad codriverGamepad = new CommandGamepad(gamepad2);
}
