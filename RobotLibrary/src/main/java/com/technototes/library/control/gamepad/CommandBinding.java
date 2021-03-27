package com.technototes.library.control.gamepad;

import com.technototes.control.gamepad.AbstractBinding;

import java.util.function.BooleanSupplier;

/** Command implementation of {@link AbstractBinding}
 * @author Alex Stedman
 */
public class CommandBinding extends AbstractBinding<CommandButton> {
    /** Create binding
     *
     * @param buttons The Buttons
     */
    public CommandBinding(BooleanSupplier... buttons){
        super(buttons);
    }
    /** Create binding
     *
     * @param buttons The Buttons
     * @param type The binding type
     */
    public CommandBinding(Type type, BooleanSupplier... buttons){
        super(type, buttons);
    }
    @Override
    public CommandButton getAsButton(AbstractBinding.Type t) {
        return new CommandButton(()->get(t));
    }
}
