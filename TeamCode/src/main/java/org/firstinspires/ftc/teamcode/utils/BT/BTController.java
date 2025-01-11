package org.firstinspires.ftc.teamcode.utils.BT;

import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BTController{

    static double threshold = 0.;
    private BooleanSupplier[] m_buttonsSuppliers;

    public final DoubleSupplier left_x;
    public final DoubleSupplier right_x;
    public final DoubleSupplier left_y;
    public final DoubleSupplier right_y;
    public final DoubleSupplier left_trigger;
    public final DoubleSupplier right_trigger;

    public BTController(Gamepad gamepad)
    {
        this.m_buttonsSuppliers = new BooleanSupplier[values().length];

        right_x= ()->gamepad.right_stick_x;
        right_y= ()->gamepad.right_stick_y;
        left_x = ()->gamepad.left_stick_x ;
        left_y = ()->gamepad.left_stick_y ;
        left_trigger = ()-> gamepad.left_trigger;
        right_trigger = ()-> gamepad.right_trigger;

        this.m_buttonsSuppliers[LEFT_X.ordinal()] = (() -> Math.abs(gamepad.left_stick_x) > threshold);
        this.m_buttonsSuppliers[LEFT_Y.ordinal()] = (() -> Math.abs(gamepad.left_stick_y) > threshold);
        this.m_buttonsSuppliers[RIGHT_X.ordinal()] = (() -> Math.abs(gamepad.right_stick_x) > threshold);
        this.m_buttonsSuppliers[RIGHT_Y.ordinal()] = (() -> Math.abs(gamepad.right_stick_y) > threshold);
        this.m_buttonsSuppliers[LEFT_TRIGGER.ordinal()] = (() -> Math.abs(gamepad.left_trigger) > threshold);
        this.m_buttonsSuppliers[RIGHT_TRIGGER.ordinal()] = (() -> Math.abs(gamepad.right_trigger) > threshold);

        this.m_buttonsSuppliers[BUMPER_LEFT.ordinal()] = ()->gamepad.left_bumper;
        this.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()] = ()->gamepad.right_bumper;
        this.m_buttonsSuppliers[DPAD_DOWN.ordinal()]=()->gamepad.dpad_down;
        this.m_buttonsSuppliers[DPAD_RIGHT.ordinal()]=()->gamepad.dpad_right;
        this.m_buttonsSuppliers[DPAD_LEFT.ordinal()]=()->gamepad.dpad_left;
        this.m_buttonsSuppliers[DPAD_UP.ordinal()]=()->gamepad.dpad_up;
        this.m_buttonsSuppliers[BUTTON_RIGHT.ordinal()]=()->gamepad.b;
        this.m_buttonsSuppliers[BUTTON_LEFT.ordinal()]=()->gamepad.x;
        this.m_buttonsSuppliers[BUTTON_UP.ordinal()]=()->gamepad.y;
        this.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]=()->gamepad.a;


    }

    public Trigger getTrigger(double threshold, Buttons... axes) {
        Trigger result = new Trigger(()->false );
        for(Buttons axis : axes ){
            result=result.or(getTrigger(axis));
        }
        return result;
    }
    private Trigger getTrigger(Buttons axis){
        return new Trigger(m_buttonsSuppliers[axis.ordinal()]);
    }
    public Trigger assignCommand(Command command, boolean cancelOnRelease, Buttons...axes){
        if(cancelOnRelease){
            return getTrigger(threshold, axes).whileActiveContinuous(command);
        }else {
            return getTrigger(threshold, axes).whenActive(command);
        }
    }

    public enum Buttons {
        BUTTON_LEFT,
        BUTTON_RIGHT,
        BUTTON_UP,
        BUTTON_DOWN,
        LEFT_X,
        LEFT_Y,
        RIGHT_X,
        RIGHT_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        TRIGGER_RIGHT,
        TRIGGER_LEFT,
        BUMPER_RIGHT,
        BUMPER_LEFT,

    }
}