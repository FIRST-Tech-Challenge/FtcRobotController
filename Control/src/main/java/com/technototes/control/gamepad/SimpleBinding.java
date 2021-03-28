package com.technototes.control.gamepad;

import java.util.function.BooleanSupplier;

/** Simple implementation of {@link AbstractBinding}
 * @author Alex Stedman
 */
public final class SimpleBinding extends AbstractBinding<GamepadButton> {
    /** Create binding
     *
     * @param buttons The buttons for the binding
     */
    public SimpleBinding(BooleanSupplier... buttons){
        super(buttons);
    }
    /** Create binding
     *
     * @param buttons The buttons for the binding
     * @param type The binding type
     */
    public SimpleBinding(Type type, BooleanSupplier... buttons){
        super(type, buttons);
    }
    @Override
    public GamepadButton getAsButton(Type t) {
        return new GamepadButton(()->get(t));
    }
}
