package com.technototes.library.control.gamepad;

import com.technototes.control.gamepad.GamepadButton;
import com.technototes.library.command.Command;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.control.Trigger;

import java.util.function.BooleanSupplier;

public class CommandButton extends GamepadButton implements GamepadTrigger<CommandButton> {
    public CommandButton(BooleanSupplier b) {
        super(b);
    }
    public CommandButton(){
        super();
    }
    @Override
    public CommandButton getInstance() {
        return this;
    }
}
