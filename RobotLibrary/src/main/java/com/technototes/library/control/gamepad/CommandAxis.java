package com.technototes.library.control.gamepad;

import com.technototes.control.gamepad.GamepadAxis;
import com.technototes.library.command.Command;
import com.technototes.library.control.Trigger;

import java.util.function.DoubleSupplier;

public class CommandAxis extends GamepadAxis implements GamepadTrigger<CommandAxis> {
    public CommandAxis(){
        super();
    }
    public CommandAxis(DoubleSupplier d){
        super(d);
    }
    public CommandAxis(DoubleSupplier d, double t){
        super(d, t);
    }

    @Override
    public CommandAxis getInstance() {
        return this;
    }
}
