package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.BTController.Axis.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;

public class BTController{

    public final Gamepad gamepad;
     static double threshold = 0.1;
     private Supplier <Boolean>[] m_axis;

    public BTController(Gamepad gamepad) {
        this.gamepad = gamepad;
        m_axis [LEFT_X.ordinal()] = (() -> Math.abs(gamepad.left_stick_x) > threshold);
        m_axis [LEFT_Y.ordinal()] = (() -> Math.abs(gamepad.left_stick_y) > threshold);
        m_axis [RIGHT_X.ordinal()] = (() -> Math.abs(gamepad.right_stick_x) > threshold);
        m_axis [RIGHT_Y.ordinal()] = (() -> Math.abs(gamepad.right_stick_y) > threshold);
        m_axis [LEFT_TRIGGER.ordinal()] = (() -> Math.abs(gamepad.left_trigger) > threshold);
        m_axis [RIGHT_TRIGGER.ordinal()] = (() -> Math.abs(gamepad.right_trigger) > threshold);

    }

    public Trigger getTrigger(double threshold, Axis... axes) {
        Trigger result = new Trigger(()->false );
        for(Axis axis : axes ){
            result=result.or(getTrigger(threshold,axis));
        }
        return result;
    }
    public Trigger assignCommand(Command command, boolean cancelOnRelease,Axis...axes){
        if(cancelOnRelease){
            return getTrigger(threshold, axes).whileActiveContinuous(command);
        }else {
            return getTrigger(threshold, axes).whenActive(command);
        }
    }

    public void setM_axis(Supplier<Boolean>[] m_axis) {
        this.m_axis = m_axis;
    }

    public enum Axis {
        LEFT_X,
        LEFT_Y,
        RIGHT_X,
        RIGHT_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER;

        Axis(Supplier<Boolean> s) {
            supplier = s;
        }

        Supplier<Boolean> supplier;


        Axis() {

        }
    }
}