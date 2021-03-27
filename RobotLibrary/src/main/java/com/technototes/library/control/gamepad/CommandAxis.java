package com.technototes.library.control.gamepad;

import com.technototes.control.gamepad.GamepadAxis;
import com.technototes.library.command.Command;
import com.technototes.library.control.Trigger;

import java.util.function.DoubleSupplier;

/** Class for command axis for the gamepad
 * @author Alex Stedman
 */
public class CommandAxis extends GamepadAxis implements GamepadTrigger<CommandAxis> {
    public CommandAxis(){
        super();
    }

    /** Make a command axis
     *
     * @param supplier The axis supplier
     */
    public CommandAxis(DoubleSupplier supplier){
        super(supplier);
    }

    /** Make a command axis
     *
     * @param supplier The axis supplier
     * @param threshold The threshold to trigger to make the axis behave as a button
     */
    public CommandAxis(DoubleSupplier supplier, double threshold){
        super(supplier, threshold);
    }

    @Override
    public CommandAxis getInstance() {
        return this;
    }

}
