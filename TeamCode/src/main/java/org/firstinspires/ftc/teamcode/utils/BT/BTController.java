package org.firstinspires.ftc.teamcode.utils.BT;

import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BTController {

    static double threshold = 0.1;
    protected BooleanSupplier[] m_buttonsSuppliers;
    protected DoubleSupplier[] m_axesSuppliers;


    public BTController(Gamepad gamepad) {
        this.m_buttonsSuppliers = new BooleanSupplier[values().length];

        this.m_axesSuppliers[Axes.RIGHT_X_axis.ordinal()] = () -> gamepad.right_stick_x;
        this.m_axesSuppliers[Axes.RIGHT_Y_axis.ordinal()] = () -> gamepad.right_stick_y;
        this.m_axesSuppliers[Axes.LEFT_X_axis.ordinal()] = () -> gamepad.left_stick_x;
        this.m_axesSuppliers[Axes.LEFT_Y_axis.ordinal()] = () -> gamepad.left_stick_y;
        this.m_axesSuppliers[Axes.LEFT_TRIGGER_axis.ordinal()] = () -> gamepad.left_trigger;
        this.m_axesSuppliers[Axes.RIGHT_TRIGGER_axis.ordinal()] = () -> gamepad.right_trigger;

        this.m_buttonsSuppliers[Buttons.LEFT_X.ordinal()] = (() -> Math.abs(gamepad.left_stick_x) > threshold);
        this.m_buttonsSuppliers[Buttons.LEFT_Y.ordinal()] = (() -> Math.abs(gamepad.left_stick_y) > threshold);
        this.m_buttonsSuppliers[Buttons.RIGHT_X.ordinal()] = (() -> Math.abs(gamepad.right_stick_x) > threshold);
        this.m_buttonsSuppliers[Buttons.RIGHT_Y.ordinal()] = (() -> Math.abs(gamepad.right_stick_y) > threshold);
        this.m_buttonsSuppliers[Buttons.LEFT_TRIGGER.ordinal()] = (() -> Math.abs(gamepad.left_trigger) > threshold);
        this.m_buttonsSuppliers[Buttons.RIGHT_TRIGGER.ordinal()] = (() -> Math.abs(gamepad.right_trigger) > threshold);

        this.m_buttonsSuppliers[BUMPER_LEFT.ordinal()] = () -> gamepad.left_bumper;
        this.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()] = () -> gamepad.right_bumper;
        this.m_buttonsSuppliers[DPAD_DOWN.ordinal()] = () -> gamepad.dpad_down;
        this.m_buttonsSuppliers[DPAD_RIGHT.ordinal()] = () -> gamepad.dpad_right;
        this.m_buttonsSuppliers[DPAD_LEFT.ordinal()] = () -> gamepad.dpad_left;
        this.m_buttonsSuppliers[DPAD_UP.ordinal()] = () -> gamepad.dpad_up;
        this.m_buttonsSuppliers[BUTTON_RIGHT.ordinal()] = () -> gamepad.b;
        this.m_buttonsSuppliers[BUTTON_LEFT.ordinal()] = () -> gamepad.x;
        this.m_buttonsSuppliers[BUTTON_UP.ordinal()] = () -> gamepad.y;
        this.m_buttonsSuppliers[BUTTON_DOWN.ordinal()] = () -> gamepad.a;


    }

    public Trigger getTrigger(Buttons... axes) {
        Trigger result = new Trigger(() -> false);
        for (Buttons axis : axes) {
            result = result.or(getTrigger(axis));
        }
        return result;
    }

    private Trigger getTrigger(Buttons axis) {
        return new Trigger(m_buttonsSuppliers[axis.ordinal()]);
    }

    public Trigger assignCommand(Command command, boolean cancelOnRelease, Buttons... axes) {
        if (cancelOnRelease) {
            return getTrigger(axes).whileActiveContinuous(command);
        } else {
            return getTrigger(axes).whenActive(command);
        }
    }

    public double getAxisValue(Axes axis) {
        return m_axesSuppliers[axis.ordinal()].getAsDouble();
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

    public enum Axes {
        LEFT_X_axis,
        RIGHT_X_axis,
        LEFT_Y_axis,
        RIGHT_Y_axis,
        LEFT_TRIGGER_axis,
        RIGHT_TRIGGER_axis
    }

}