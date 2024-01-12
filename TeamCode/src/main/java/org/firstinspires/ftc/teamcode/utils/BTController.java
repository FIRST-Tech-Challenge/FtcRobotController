package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BTController{

    public final Gamepad gamepad1;
    public final Gamepad gamepad2;

    static double threshold = 0.1;
    private BooleanSupplier[] m_buttonsSuppliers;

    public final DoubleSupplier left_x1;
    public final DoubleSupplier left_x2;
    public final DoubleSupplier left_y1;
    public final DoubleSupplier left_y2;

    public BTController(Gamepad gamepad1, Gamepad gamepad2, BooleanSupplier[] m_buttonsSuppliers, DoubleSupplier left_x1, DoubleSupplier left_x2, DoubleSupplier left_y1, DoubleSupplier left_y2)
    {
        this.m_buttonsSuppliers = m_buttonsSuppliers;
        this.left_x1 = left_x1;
        this.left_x2 = left_x2;
        this.left_y1 = left_y1;
        this.left_y2 = left_y2;
        this.gamepad1=gamepad1;
        this.gamepad2=gamepad2;

        left_x1=()->gamepad1.left_stick_x;
        left_x2=()->gamepad2.left_stick_x;
        this.m_buttonsSuppliers[LEFT_X1.ordinal()] = (() -> Math.abs(gamepad1.left_stick_x) > threshold);
        this.m_buttonsSuppliers[LEFT_Y1.ordinal()] = (() -> Math.abs(gamepad1.left_stick_y) > threshold);
        this.m_buttonsSuppliers[RIGHT_X1.ordinal()] = (() -> Math.abs(gamepad1.right_stick_x) > threshold);
        this.m_buttonsSuppliers[RIGHT_Y1.ordinal()] = (() -> Math.abs(gamepad1.right_stick_y) > threshold);
        this.m_buttonsSuppliers[LEFT_TRIGGER1.ordinal()] = (() -> Math.abs(gamepad1.left_trigger) > threshold);
        this.m_buttonsSuppliers[RIGHT_TRIGGER1.ordinal()] = (() -> Math.abs(gamepad1.right_trigger) > threshold);

        this.m_buttonsSuppliers[BUMPER_LEFT1.ordinal()] = ()->gamepad1.left_bumper;
        this.m_buttonsSuppliers[DPAD_DOWN1.ordinal()]=()->gamepad1.dpad_down;
        this.m_buttonsSuppliers[DPAD_RIGHT1.ordinal()]=()->gamepad1.dpad_right;
        this.m_buttonsSuppliers[DPAD_LEFT1.ordinal()]=()->gamepad1.dpad_left;
        this.m_buttonsSuppliers[DPAD_UP1.ordinal()]=()->gamepad1.dpad_up;
        this.m_buttonsSuppliers[BUTTON_RIGHT1.ordinal()]=()->gamepad1.b;
        this.m_buttonsSuppliers[BUTTON_LEFT1.ordinal()]=()->gamepad1.x;
        this.m_buttonsSuppliers[BUTTON_UP1.ordinal()]=()->gamepad1.y;
        this.m_buttonsSuppliers[BUTTON_DOWN1.ordinal()]=()->gamepad1.a;


        this.m_buttonsSuppliers[LEFT_X2.ordinal()] = (() -> Math.abs(gamepad2.left_stick_x) > threshold);
        this.m_buttonsSuppliers[LEFT_Y2.ordinal()] = (() -> Math.abs(gamepad2.left_stick_y) > threshold);
        this.m_buttonsSuppliers[RIGHT_X2.ordinal()] = (() -> Math.abs(gamepad2.right_stick_x) > threshold);
        this.m_buttonsSuppliers[RIGHT_Y2.ordinal()] = (() -> Math.abs(gamepad2.right_stick_y) > threshold);
        this.m_buttonsSuppliers[LEFT_TRIGGER2.ordinal()] = (() -> Math.abs(gamepad2.left_trigger) > threshold);
        this.m_buttonsSuppliers[RIGHT_TRIGGER2.ordinal()] = (() -> Math.abs(gamepad2.right_trigger) > threshold);

        this.m_buttonsSuppliers[BUMPER_LEFT2.ordinal()] = ()->gamepad2.left_bumper;
        this.m_buttonsSuppliers[DPAD_DOWN2.ordinal()]=()->gamepad2.dpad_down;
        this.m_buttonsSuppliers[DPAD_RIGHT2.ordinal()]=()->gamepad2.dpad_right;
        this.m_buttonsSuppliers[DPAD_LEFT2.ordinal()]=()->gamepad2.dpad_left;
        this.m_buttonsSuppliers[DPAD_UP2.ordinal()]=()->gamepad2.dpad_up;
        this.m_buttonsSuppliers[BUTTON_RIGHT2.ordinal()]=()->gamepad2.b;
        this.m_buttonsSuppliers[BUTTON_LEFT2.ordinal()]=()->gamepad2.x;
        this.m_buttonsSuppliers[BUTTON_UP2.ordinal()]=()->gamepad2.y;
        this.m_buttonsSuppliers[BUTTON_DOWN2.ordinal()]=()->gamepad2.a;




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
        BUTTON_LEFT1,
        BUTTON_RIGHT1,
        BUTTON_UP1,
        BUTTON_DOWN1,
        LEFT_X1,
        LEFT_Y1,
        RIGHT_X1,
        RIGHT_Y1,
        LEFT_TRIGGER1,
        RIGHT_TRIGGER1,
        DPAD_UP1,
        DPAD_DOWN1,
        DPAD_LEFT1,
        DPAD_RIGHT1,
        TRIGGER_RIGHT1,
        TRIGGER_LEFT1,
        BUMPER_RIGHT1,
        BUMPER_LEFT1,
        BUTTON_LEFT2,
        BUTTON_RIGHT2,
        BUTTON_UP2,
        BUTTON_DOWN2,
        LEFT_X2,
        LEFT_Y2,
        RIGHT_X2,
        RIGHT_Y2,
        LEFT_TRIGGER2,
        RIGHT_TRIGGER2,
        DPAD_UP2,
        DPAD_DOWN2,
        DPAD_LEFT2,
        DPAD_RIGHT2,
        TRIGGER_RIGHT2,
        TRIGGER_LEFT2,
        BUMPER_RIGHT2,
        BUMPER_LEFT2;

    }
}