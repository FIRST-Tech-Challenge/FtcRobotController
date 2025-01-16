package org.firstinspires.ftc.TBlib;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import java.util.function.BooleanSupplier;


public class TBToggle {

    private boolean value;

    private Trigger trigger;

    public TBToggle(BooleanSupplier btn, boolean defaultVal){
        trigger = new Trigger(btn);
        value = defaultVal;
        trigger.whenActive(new InstantCommand(()->value = !value));
    }

    public Trigger getAsTrigger(){
        return new Trigger(()->value);
    }
}
