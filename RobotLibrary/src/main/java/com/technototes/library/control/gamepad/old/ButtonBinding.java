package com.technototes.library.control.gamepad.old;

import java.util.function.BooleanSupplier;
@Deprecated
public class ButtonBinding extends ButtonGamepadComponent {
    public BooleanSupplier[] bindings;

    public ButtonBinding(BooleanSupplier... b) {
        super(() -> {
            for (BooleanSupplier b2 : b) {
                if (!b2.getAsBoolean()) {
                    return false;
                }
            }
            return true;
        });
        bindings = b;
    }

    //DEFAULT FOR USING THIS AS NORMAL BINDING
    public ButtonGamepadComponent allActive() {
        return this;
    }

    public ButtonGamepadComponent oneActive() {
        return new ButtonGamepadComponent(() -> {
            for (BooleanSupplier b : bindings) {
                if (b.getAsBoolean()) {
                    return true;
                }
            }
            return false;
        });
    }

    public ButtonGamepadComponent noneActive() {
        return new ButtonGamepadComponent(() -> {
            for (BooleanSupplier b : bindings) {
                if (b.getAsBoolean()) {
                    return false;
                }
            }
            return true;
        });
    }
}
