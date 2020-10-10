package com.technototes.library.control;

import com.technototes.control.gamepad.GamepadButton;
import com.technototes.library.command.Command;

public interface Trigger<T> {
    T whenActivated(Command c);

    T whenDeactivated(Command c);

    T whileActivated(Command c);

    T whileDeactivated(Command c);

    T whenToggled(Command c);

    T whenInverseToggled(Command c);

    T whileToggled(Command c);

    T whileInverseToggled(Command c);

}
