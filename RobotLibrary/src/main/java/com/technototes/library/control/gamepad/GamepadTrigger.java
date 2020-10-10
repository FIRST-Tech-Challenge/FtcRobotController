package com.technototes.library.control.gamepad;

import com.technototes.control.gamepad.GamepadButton;
import com.technototes.library.command.Command;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.control.Trigger;

import java.util.function.BooleanSupplier;

public interface GamepadTrigger<T extends GamepadButton> extends Trigger<T> {

    @Override
    default T whenActivated(Command c){
        return schedule(getInstance()::isJustActivated, c);
    }

    @Override
    default T whenDeactivated(Command c){
        return schedule(getInstance()::isDeactivated, c);
    }

    @Override
    default T whileActivated(Command c){
        return schedule(getInstance()::isActivated, c);
    }

    @Override
    default T whileDeactivated(Command c){
        return schedule(getInstance()::isDeactivated, c);
    }

    @Override
    default T whenToggled(Command c){
        return schedule(getInstance()::isJustToggled, c);
    }

    @Override
    default T whenInverseToggled(Command c){
        return schedule(getInstance()::isJustInverseToggled, c);
    }

    @Override
    default T whileToggled(Command c){
        return schedule(getInstance()::isToggled, c);
    }

    @Override
    default T whileInverseToggled(Command c){
        return schedule(getInstance()::isInverseToggled, c);
    }

    T getInstance();

    default T schedule(BooleanSupplier condition, Command command){
        CommandScheduler.getRunInstance().schedule(condition, command);
        return getInstance();
    }

}
