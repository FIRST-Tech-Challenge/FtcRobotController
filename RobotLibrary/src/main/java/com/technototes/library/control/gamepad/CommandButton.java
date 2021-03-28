package com.technototes.library.control.gamepad;

import com.technototes.control.gamepad.GamepadButton;

import java.util.function.BooleanSupplier;

/** Class for command buttons for gamepad
 * @author Alex Stedman
 */
public class CommandButton extends GamepadButton implements GamepadInput<CommandButton> {
    /** Make command button
     *
     * @param supplier The supplier for the button
     */
    public CommandButton(BooleanSupplier supplier) {
        super(supplier);
    }

    public CommandButton(){
        super();
    }

    @Override
    public CommandButton getInstance() {
        return this;
    }
}
