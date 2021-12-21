package org.commandftc;

import com.qualcomm.robotcore.hardware.Gamepad;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class represents an FTC gamepad that supports the use of lambda expressions to assign commands.
 */
public class Gp {
    private Gamepad gamepad;
    public Gp(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public Button a() {
        return new Button(() -> gamepad.a);
    }

    public Button b() {
        return new Button(() -> gamepad.b);
    }

    public Button back() {
        return new Button(() -> gamepad.back);
    }

    public Button dpad_down() {
        return new Button(() -> gamepad.dpad_down);
    }

    public Button dpad_up() {
        return new Button(() -> gamepad.dpad_up);
    }

    public Button dpad_left() {
        return new Button(() -> gamepad.dpad_left);
    }

    public Button dpad_right() {
        return new Button(() -> gamepad.dpad_right);
    }

    public Button left_bumper() {
        return new Button(() -> gamepad.left_bumper);
    }

    public Button left_stick_button() {
        return new Button(() -> gamepad.left_stick_button);
    }

    public Button right_bumper() {
        return new Button(() -> gamepad.right_bumper);
    }

    public Button right_stick_button() {
        return new Button(() -> gamepad.right_stick_button);
    }

    public Button x() {
        return new Button(() -> gamepad.x);
    }

    public Button y() {
        return new Button(() -> gamepad.y);
    }

    public double left_stick_x() { return gamepad.left_stick_x; }
    public double left_stick_y() { return gamepad.left_stick_y; }
    public double right_stick_x() { return gamepad.right_stick_x; }
    public double right_stick_y() { return gamepad.right_stick_y; }

    public double left_trigger() {return gamepad.left_trigger;}
    public double right_trigger() {return gamepad.right_trigger;}
}