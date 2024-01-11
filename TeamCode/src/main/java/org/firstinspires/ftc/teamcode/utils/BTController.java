package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BTController{

    public final Gamepad gamepad;
    static double threshold = 0.1;
    private BooleanSupplier[] m_buttonsSuppliers;

    public final DoubleSupplier left_x;//todo: add more for each of the axis

    public BTController(Gamepad gamepad)
    {
        this.gamepad=gamepad;
        left_x=()->gamepad.left_stick_x;
        m_buttonsSuppliers[LEFT_X.ordinal()] = (() -> Math.abs(gamepad.left_stick_x) > threshold);
        m_buttonsSuppliers[LEFT_Y.ordinal()] = (() -> Math.abs(gamepad.left_stick_y) > threshold);
        m_buttonsSuppliers[RIGHT_X.ordinal()] = (() -> Math.abs(gamepad.right_stick_x) > threshold);
        m_buttonsSuppliers[RIGHT_Y.ordinal()] = (() -> Math.abs(gamepad.right_stick_y) > threshold);
        m_buttonsSuppliers[LEFT_TRIGGER.ordinal()] = (() -> Math.abs(gamepad.left_trigger) > threshold);
        m_buttonsSuppliers[RIGHT_TRIGGER.ordinal()] = (() -> Math.abs(gamepad.right_trigger) > threshold);

        m_buttonsSuppliers[BUMPER_LEFT.ordinal()] = ()->gamepad.left_bumper;
        m_buttonsSuppliers[DPAD_DOWN.ordinal()]=()->gamepad.dpad_down;
        m_buttonsSuppliers[DPAD_RIGHT.ordinal()]=()->gamepad.dpad_right;
        m_buttonsSuppliers[DPAD_LEFT.ordinal()]=()->gamepad.dpad_left;
        m_buttonsSuppliers[DPAD_UP.ordinal()]=()->gamepad.dpad_up;
        m_buttonsSuppliers[BUTTON_RIGHT.ordinal()]=()->gamepad.b;
        m_buttonsSuppliers[BUTTON_LEFT.ordinal()]=()->gamepad.x;
        m_buttonsSuppliers[BUTTON_UP.ordinal()]=()->gamepad.y;
        m_buttonsSuppliers[BUTTON_DOWN.ordinal()]=()->gamepad.a;




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
        BUMPER_LEFT;

    }
}